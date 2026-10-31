#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Bearing-only formation of ten ESP-drones in Gazebo (CCE 2025).

Drones 1 and 2 are leaders with the velocities of the paper. The other eight
only use bearings to their neighbors, with the Super-Twisting law (sta) or the
baseline proportional-derivative law (pd).

The ESP-drone is much smaller than the Bebop 2 of the paper, so positions and
speeds are multiplied by --scale. Bearings do not change with this scale.

Usage:
    python3 scripts/fly_formation.py --controller sta           # with control panel
    python3 scripts/fly_formation.py --controller sta --auto --headless --output results
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node
import numpy as np

REPO = Path(__file__).resolve().parent.parent
WORLD = 'espdrone_multi'
N = 10
LEADERS = (0, 1)
DEFAULT_SCALE = 0.5
TAKEOFF_SEC = 8.0
LAND_SEC = 8.0
VELOCITY_FILTER_SEC = 0.05
MAX_SPEED = 1.0
NU_LIMIT = 2.0

# Positions of the paper, in meters.
START = np.array([
    [1.0, 1.0, 5.0], [-1.0, 1.0, 5.0], [-2.52, -0.7, 5.0], [2.1, -1.88, 5.0], [2.72, -0.7, 5.0],
    [-0.1, 1.2, 5.0], [-1.52, -0.7, 5.0], [0.5, -1.88, 5.0], [1.72, -0.7, 5.0], [1.8, 1.2, 5.0]])
TARGET = np.array([
    [1.0, 1.0, 3.0], [-1.0, 1.0, 3.0], [-1.62, -0.9, 3.0], [0.0, -2.08, 3.0], [1.62, -0.9, 3.0],
    [-1.0, 1.0, 1.0], [-1.62, -0.9, 1.0], [0.0, -2.08, 1.0], [1.62, -0.9, 1.0], [1.0, 1.0, 1.0]])
# Observer -> target, numbered as in the paper.
EDGES = [(3, 2), (3, 4), (4, 5), (4, 1), (4, 2), (5, 4), (5, 1), (5, 10), (6, 10), (6, 1),
         (6, 8), (7, 10), (7, 6), (7, 8), (7, 2), (8, 10), (8, 9), (8, 7), (8, 6), (8, 3),
         (9, 10), (9, 1), (9, 2), (10, 7), (10, 8), (10, 6), (10, 5), (10, 9)]
# Gains per axis (x and y, z), tuned in flight for this drone and scale. The STA law
# has k0, k1, k2 and the PD law has k0, k1.
GAINS = {
    'sta': {'xy': (0.29439, 0.39197, 0.00020695), 'z': (0.18682, 0.1075, 0.0013275)},
    'pd': {'xy': (0.62039, 0.78873), 'z': (5.60651, 1.1358)},
}


def leader_velocity(t, leader):
    """Leader velocity of the paper; leader 1 changes during two intervals."""
    vy = 0.075 * math.cos(0.05 * t) + 0.075 * math.sin(2.0 * t)
    drift = 0.075 * math.sin(0.05 * t)
    vx = 0.03 + drift
    if leader == 0 and 20.0 < t <= 24.0:
        vx = 0.018 + 0.012 * math.cos(math.pi * (t - 20.0) / 2.0) + drift
    elif leader == 0 and 50.0 < t <= 54.0:
        vx = 0.06 - 0.03 * math.cos(math.pi * (t - 50.0) / 2.0) + drift
    return np.array([vx, vy, 0.048])


def make_world(scale):
    """Return the path of a world file with ten drones at the scaled start."""
    template = (REPO / 'worlds' / 'espdrone_multi.sdf').read_text()
    head, _, rest = template.partition('    <include>')
    drone = '    <include>' + rest.partition('</include>')[0] + '</include>\n'
    tail = rest.rpartition('</include>')[2]
    parts = [head]
    for i in range(N):
        block = drone.replace('espdrone1', f'espdrone{i + 1}')
        x, y = START[i, :2] * scale
        block = block.replace('<pose>0.4 0.4 0.3 0 0 0</pose>', f'<pose>{x} {y} 0.05 0 0 0</pose>')
        parts.append(block)
    parts.append(tail)
    world = tempfile.NamedTemporaryFile('w', suffix='_cce2025.sdf', delete=False)
    world.write(''.join(parts))
    world.close()
    return world.name


class Formation:
    """Gazebo process, the bearing-only law and the telemetry of the formation."""

    IDLE, TAKING_OFF, AUTOMATIC, LANDING, STOPPED = range(5)
    LABELS = {IDLE: ('IDLE', '#555555'), TAKING_OFF: ('TAKING OFF', '#2471a3'),
              AUTOMATIC: ('AUTOMATIC', '#1e8449'), LANDING: ('LANDING', '#b9770e'),
              STOPPED: ('EMERGENCY STOP', '#c0392b')}

    def __init__(self, controller, headless, scale, gains=None):
        self.controller = controller
        self.scale = scale
        self.gains = gains or GAINS[controller]
        self.lock = threading.Lock()
        self.state = self.IDLE
        self.pos = np.full((N, 3), np.nan)
        self.vel = np.zeros((N, 3))
        self.sim_time = 0.0
        self.t_state = 0.0
        self.last_stamp = None
        self.last_law = None
        self.nu = np.zeros((N, 3))
        self.cmd = np.zeros((N, 3))
        self.log = []
        self.steps = 0
        self.running = True
        desired = TARGET * scale
        self.edges = [(a - 1, b - 1) for a, b in EDGES]
        self.g_des = {(a, b): (desired[b] - desired[a]) / np.linalg.norm(desired[b] - desired[a])
                      for a, b in self.edges}
        self.node = Node()
        self.pub_pos = [self.node.advertise(f'/espdrone{i + 1}/cmd_position', Pose) for i in range(N)]
        self.pub_vel = [self.node.advertise(f'/espdrone{i + 1}/cmd_vel_world', Twist) for i in range(N)]
        self.pub_enable = [self.node.advertise(f'/espdrone{i + 1}/motor_enable', Boolean)
                           for i in range(N)]
        self.names = {f'espdrone{i + 1}': i for i in range(N)}
        self.node.subscribe(Pose_V, f'/world/{WORLD}/pose/info', self._on_poses)
        self.world_file = make_world(scale)
        env = dict(os.environ)
        env['GZ_SIM_RESOURCE_PATH'] = f"{REPO / 'models'}:{env.get('GZ_SIM_RESOURCE_PATH', '')}"
        env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = (
            f"{REPO / 'build'}:{env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}")
        cmd = ['gz', 'sim', '-r'] + (['-s'] if headless else []) + [self.world_file]
        self.proc = subprocess.Popen(cmd, env=env, start_new_session=True,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _on_poses(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        pos = np.full((N, 3), np.nan)
        for pose in msg.pose:
            i = self.names.get(pose.name)
            if i is not None:
                pos[i] = (pose.position.x, pose.position.y, pose.position.z)
        with self.lock:
            if self.last_stamp is not None and stamp > self.last_stamp:
                dt = stamp - self.last_stamp
                raw = (pos - self.pos) / dt
                alpha = dt / (VELOCITY_FILTER_SEC + dt)
                self.vel = np.where(np.isnan(raw), self.vel, self.vel + alpha * (raw - self.vel))
            self.pos = pos
            self.sim_time = stamp
            self.last_stamp = stamp

    def wait_ready(self, timeout=90.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                if not np.isnan(self.pos).any():
                    return True
            time.sleep(0.2)
        return False

    def set_state(self, state):
        with self.lock:
            if state == self.LANDING:
                self.land_from = self.pos.copy()
            if state == self.AUTOMATIC:
                self.nu[:] = 0.0
                self.cmd[:] = 0.0
                self.last_law = None
            self.state = state
            self.t_state = self.sim_time
        self._send_enable()

    def _send_enable(self):
        enable = Boolean()
        enable.data = self.state in (self.TAKING_OFF, self.AUTOMATIC, self.LANDING)
        for pub in self.pub_enable:
            pub.publish(enable)

    def _send_positions(self, targets):
        for pub, target in zip(self.pub_pos, targets):
            msg = Pose()
            msg.position.x, msg.position.y, msg.position.z = (float(v) for v in target)
            msg.orientation.w = 1.0
            pub.publish(msg)

    def _send_velocities(self, velocities):
        for pub, velocity in zip(self.pub_vel, velocities):
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.linear.z = (float(v) for v in velocity)
            pub.publish(msg)

    def _bearing_law(self, pos, vel, dt):
        """Acceleration of each follower from its bearings, as in the paper."""
        accel = np.zeros((N, 3))
        errors = []
        for a, b in self.edges:
            rel = pos[b] - pos[a]
            dist = np.linalg.norm(rel)
            if dist < 1e-6:
                continue
            g = rel / dist
            g_rate = (np.eye(3) - np.outer(g, g)) @ (vel[b] - vel[a]) / dist
            e = g - self.g_des[(a, b)]
            errors.append(e)
            if a in LEADERS:
                continue
            for axis in range(3):
                k = self.gains['z' if axis == 2 else 'xy']
                if self.controller == 'pd':
                    accel[a, axis] += k[0] * e[axis] + k[1] * g_rate[axis]
                else:
                    s = -g_rate[axis] - k[0] * e[axis]
                    self.nu[a, axis] = np.clip(self.nu[a, axis] - k[2] * np.sign(s) * dt,
                                               -NU_LIMIT, NU_LIMIT)
                    accel[a, axis] += self.nu[a, axis] - k[1] * math.sqrt(abs(s)) * np.sign(s)
        return accel, np.array(errors)

    def step(self):
        with self.lock:
            state, now, pos, vel, t_state = (self.state, self.sim_time, self.pos.copy(),
                                             self.vel.copy(), self.t_state)
        dt_state = now - t_state
        # An enable message sent just after Gazebo starts can be lost, and the
        # motors would never start. Repeat it while flying.
        self.steps += 1
        if self.steps % 10 == 0:
            self._send_enable()
        if state == self.TAKING_OFF:
            target = START * self.scale
            target[:, 2] *= min(1.0, dt_state / TAKEOFF_SEC)
            self._send_positions(target)
        elif state == self.AUTOMATIC:
            if self.last_law is None:
                self.last_law = now
            dt = now - self.last_law
            self.last_law = now
            if dt <= 0.0 or dt > 0.5:
                return
            accel, errors = self._bearing_law(pos, vel, dt)
            self.cmd = np.clip(self.cmd + accel * dt, -MAX_SPEED, MAX_SPEED)
            for leader in LEADERS:
                self.cmd[leader] = leader_velocity(dt_state, leader) * self.scale
            self._send_velocities(self.cmd)
            metric = float(np.sqrt(np.sum(errors ** 2) / (3 * (N - len(LEADERS)))))
            self.log.append((dt_state, metric, *pos.ravel()))
        elif state == self.LANDING:
            s = max(0.0, 1.0 - dt_state / LAND_SEC)
            target = self.land_from.copy()
            target[:, 2] *= s
            self._send_positions(target)
            if s == 0.0:
                self.set_state(self.IDLE)

    def run_commands(self):
        while self.running:
            self.step()
            time.sleep(0.01)

    def close(self):
        self.running = False
        try:
            os.killpg(self.proc.pid, signal.SIGTERM)
            self.proc.wait(timeout=8)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        os.unlink(self.world_file)


def wait_sim(formation, seconds):
    with formation.lock:
        end = formation.sim_time + seconds
    while True:
        with formation.lock:
            if formation.sim_time >= end:
                return
        if formation.proc.poll() is not None:
            raise RuntimeError('Gazebo stopped')
        time.sleep(0.05)


def save_log(formation, output, tag):
    os.makedirs(output, exist_ok=True)
    log = np.array(formation.log)
    names = [f'{c}{i + 1}' for i in range(N) for c in 'xyz']
    with open(os.path.join(output, f'{tag}.csv'), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'e_m'] + names)
        writer.writerows(np.round(log, 5).tolist())
    settled = log[log[:, 0] >= 30.0, 1]
    return float(settled.min()), float(np.median(settled)), float(settled.max())


def run_auto(args):
    formation = Formation(args.controller, args.headless, args.scale, json.loads(args.gains) or None)
    threading.Thread(target=formation.run_commands, daemon=True).start()
    try:
        if not formation.wait_ready():
            raise RuntimeError('Gazebo did not start. Check that the plugins were built.')
        wait_sim(formation, 1.0)
        formation.set_state(Formation.TAKING_OFF)
        wait_sim(formation, TAKEOFF_SEC + 3.0)
        formation.set_state(Formation.AUTOMATIC)
        wait_sim(formation, args.duration)
        low, mid, high = save_log(formation, args.output, args.controller)
        print(f'{args.controller}: bearing error metric after 30 s: minimum {low:.4f}, '
              f'median {mid:.4f}, maximum {high:.4f}', flush=True)
    finally:
        formation.close()


def run_panel(args):
    import tkinter as tk
    from tkinter import ttk

    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    formation = Formation(args.controller, args.headless, args.scale, json.loads(args.gains) or None)
    threading.Thread(target=formation.run_commands, daemon=True).start()

    root = tk.Tk()
    root.title(f'CCE 2025 | {args.controller.upper()}')
    root.geometry('1200x800')
    style = ttk.Style()
    style.theme_use('clam')
    style.configure('TButton', font=('Arial', 11), padding=8)
    style.configure('Stop.TButton', font=('Arial', 11, 'bold'), foreground='#c0392b')
    panel = ttk.Frame(root, padding=12, relief='groove')
    panel.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 4), pady=8)
    ttk.Label(panel, text='CCE 2025', font=('Arial', 16, 'bold')).pack()
    ttk.Label(panel, text=f'{args.controller.upper()} bearing-only control',
              font=('Arial', 11)).pack(pady=(2, 10))
    status = tk.Label(panel, text='STARTING', font=('Arial', 12, 'bold'))
    status.pack(pady=8)
    for text, state, btn_style in (
            ('Take Off', Formation.TAKING_OFF, 'TButton'),
            ('Automatic Mode', Formation.AUTOMATIC, 'TButton'),
            ('Land', Formation.LANDING, 'TButton'),
            ('Emergency Stop', Formation.STOPPED, 'Stop.TButton')):
        ttk.Button(panel, text=text, width=20, style=btn_style,
                   command=lambda s=state: formation.set_state(s)).pack(pady=6, padx=8)

    frame = ttk.Frame(root, padding=4)
    frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 8), pady=8)
    fig = Figure(figsize=(9, 8), dpi=100, constrained_layout=True)
    grid = fig.add_gridspec(5, 1)
    ax_3d = fig.add_subplot(grid[:3, 0], projection='3d')
    ax_err = fig.add_subplot(grid[3:, 0])
    canvas = FigureCanvasTkAgg(fig, master=frame)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def draw():
        with formation.lock:
            pos, state = formation.pos.copy(), formation.state
        log = list(formation.log[-6000:])
        label, color = Formation.LABELS[state]
        status.config(text=label, fg=color)
        if not np.isnan(pos).any():
            elev, azim = ax_3d.elev, ax_3d.azim
            ax_3d.cla()
            ax_3d.view_init(elev=elev, azim=azim)
            ax_3d.set_title('Formation', fontsize=11)
            ax_3d.set_xlabel('x [m]', fontsize=8)
            ax_3d.set_ylabel('y [m]', fontsize=8)
            ax_3d.set_zlabel('z [m]', fontsize=8)
            for a, b in formation.edges:
                ax_3d.plot(*zip(pos[a], pos[b]), color='#5dade2', linewidth=0.6, alpha=0.5)
            ax_3d.scatter(*pos[2:].T, color='#2471a3', s=30)
            ax_3d.scatter(*pos[:2].T, color='#c0392b', s=60, marker='s')
            center = pos.mean(axis=0)
            half = max(1.5 * args.scale * 2, float(np.abs(pos - center).max()) + 0.2)
            ax_3d.set_xlim(center[0] - half, center[0] + half)
            ax_3d.set_ylim(center[1] - half, center[1] + half)
            ax_3d.set_zlim(max(0.0, center[2] - half), center[2] + half)
        ax_err.cla()
        ax_err.set_title('Bearing error', fontsize=11)
        ax_err.set_xlabel('time in automatic mode [s]', fontsize=9)
        ax_err.set_ylabel('$e_m$', fontsize=9)
        ax_err.grid(True, linewidth=0.5, linestyle='--', alpha=0.5)
        if log:
            data = np.array([row[:2] for row in log])
            ax_err.semilogy(data[:, 0], data[:, 1], color='#1e8449', linewidth=1.2)
        canvas.draw_idle()
        root.after(250, draw)

    root.protocol('WM_DELETE_WINDOW', root.quit)
    signal.signal(signal.SIGINT, lambda *_: root.quit())

    def keep_alive():
        root.after(200, keep_alive)
    keep_alive()
    root.after(250, draw)
    try:
        root.mainloop()
    finally:
        formation.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--controller', choices=tuple(GAINS), default='sta')
    parser.add_argument('--scale', type=float, default=DEFAULT_SCALE,
                        help='size of the formation and speeds; 1.0 is the size of the paper')
    parser.add_argument('--headless', action='store_true', help='no Gazebo window')
    parser.add_argument('--auto', action='store_true', help='fly without the control panel')
    parser.add_argument('--duration', type=float, default=60.0, help='automatic mode [s]')
    parser.add_argument('--gains', default='{}', help='gains as JSON, for tuning')
    parser.add_argument('--output', default='results')
    args = parser.parse_args()
    if not shutil.which('gz'):
        raise SystemExit('Gazebo was not found. See the installation steps in README.md.')
    if args.auto:
        run_auto(args)
    else:
        run_panel(args)


if __name__ == '__main__':
    main()
