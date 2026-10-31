# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Formations for the bearing-only laws: the one of the paper, pyramids and letters.

A scenario has the desired positions, the sensing graph, the pads on the ground and
the point where each drone ends its take-off. The first two drones are the leaders.
The graph is grown until the followers are fixed by the bearings and the leaders,
which happens when the bearing Laplacian of the followers has no zero eigenvalue.
"""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import linear_sum_assignment

PAPER_START = np.array([
    [1.0, 1.0, 5.0], [-1.0, 1.0, 5.0], [-2.52, -0.7, 5.0], [2.1, -1.88, 5.0], [2.72, -0.7, 5.0],
    [-0.1, 1.2, 5.0], [-1.52, -0.7, 5.0], [0.5, -1.88, 5.0], [1.72, -0.7, 5.0], [1.8, 1.2, 5.0]])
PAPER_TARGET = np.array([
    [1.0, 1.0, 3.0], [-1.0, 1.0, 3.0], [-1.62, -0.9, 3.0], [0.0, -2.08, 3.0], [1.62, -0.9, 3.0],
    [-1.0, 1.0, 1.0], [-1.62, -0.9, 1.0], [0.0, -2.08, 1.0], [1.62, -0.9, 1.0], [1.0, 1.0, 1.0]])
# Observer -> target, numbered as in the paper.
PAPER_EDGES = [(3, 2), (3, 4), (4, 5), (4, 1), (4, 2), (5, 4), (5, 1), (5, 10), (6, 10), (6, 1),
               (6, 8), (7, 10), (7, 6), (7, 8), (7, 2), (8, 10), (8, 9), (8, 7), (8, 6), (8, 3),
               (9, 10), (9, 1), (9, 2), (10, 7), (10, 8), (10, 6), (10, 5), (10, 9)]

# Rows of a 5x7 font, top to bottom, with the most significant of 5 bits on the left.
FONT = {
    'A': (14, 17, 17, 31, 17, 17, 17), 'B': (30, 17, 17, 30, 17, 17, 30),
    'C': (14, 17, 16, 16, 16, 17, 14), 'D': (30, 17, 17, 17, 17, 17, 30),
    'E': (31, 16, 16, 30, 16, 16, 31), 'F': (31, 16, 16, 30, 16, 16, 16),
    'G': (14, 17, 16, 23, 17, 17, 15), 'H': (17, 17, 17, 31, 17, 17, 17),
    'I': (14, 4, 4, 4, 4, 4, 14), 'J': (7, 2, 2, 2, 2, 18, 12),
    'K': (17, 18, 20, 24, 20, 18, 17), 'L': (16, 16, 16, 16, 16, 16, 31),
    'M': (17, 27, 21, 21, 17, 17, 17), 'N': (17, 25, 21, 19, 17, 17, 17),
    'O': (14, 17, 17, 17, 17, 17, 14), 'P': (30, 17, 17, 30, 16, 16, 16),
    'Q': (14, 17, 17, 17, 21, 18, 13), 'R': (30, 17, 17, 30, 20, 18, 17),
    'S': (15, 16, 16, 14, 1, 1, 30), 'T': (31, 4, 4, 4, 4, 4, 4),
    'U': (17, 17, 17, 17, 17, 17, 14), 'V': (17, 17, 17, 17, 17, 10, 4),
    'W': (17, 17, 17, 21, 21, 27, 17), 'X': (17, 17, 10, 4, 10, 17, 17),
    'Y': (17, 17, 10, 4, 4, 4, 4), 'Z': (31, 1, 2, 4, 8, 16, 31),
}
BASE_HEIGHT = 1.0
PAD_SPACING = 0.6
START_NOISE = 0.1
MAX_TAKEOFF_SPEED = 0.5
HOP_HEIGHT = 2.5
CLIMB_SEC = 8.0


@dataclass
class Scenario:
    target: np.ndarray
    start: np.ndarray
    pads: np.ndarray
    edges: list
    climb_sec: float
    travel_sec: float

    @property
    def size(self):
        return len(self.target)

    @property
    def takeoff_sec(self):
        return self.climb_sec + self.travel_sec

    def _above_pads(self, pads):
        return pads + np.array([0.0, 0.0, HOP_HEIGHT if self.climb_sec > 0.0 else 0.0])

    def takeoff_point(self, elapsed):
        """Straight up first: a drone that moves sideways near the ground tips over."""
        lifted = self._above_pads(self.pads)
        if elapsed < self.climb_sec:
            return self.pads + (elapsed / self.climb_sec) * (lifted - self.pads)
        s = min(1.0, (elapsed - self.climb_sec) / self.travel_sec)
        return lifted + s * (self.start - lifted)

    def landing_point(self, elapsed, origin, pads):
        """The take-off backwards: above the pads first, then straight down."""
        lifted = self._above_pads(pads)
        if elapsed < self.travel_sec:
            return origin + (elapsed / self.travel_sec) * (lifted - origin)
        s = min(1.0, (elapsed - self.travel_sec) / max(self.climb_sec, 1e-6))
        return lifted + s * (pads - lifted)


def pyramid(levels, spacing):
    """Square layers of 1, 4, 9, ... drones, with the largest one at the bottom."""
    points = []
    for layer, side in enumerate(range(levels, 0, -1)):
        for i in range(side):
            for j in range(side):
                points.append(((i - (side - 1) / 2) * spacing, (j - (side - 1) / 2) * spacing,
                               BASE_HEIGHT + layer * spacing))
    return np.array(points)


def letters(text, spacing):
    """Letters of a 5x7 font on the vertical plane y = 0."""
    points = []
    for k, letter in enumerate(text.upper()):
        if letter not in FONT:
            raise ValueError(f'No letter {letter!r} in the font: use A to Z.')
        for row, bits in enumerate(FONT[letter]):
            for col in range(5):
                if bits >> (4 - col) & 1:
                    points.append(((k * 6 + col) * spacing, 0.0,
                                   BASE_HEIGHT + (6 - row) * spacing))
    points = np.array(points)
    points[:, 0] -= points[:, 0].mean()
    return points


def leaders_first(points):
    """Put the two farthest drones first: they become the leaders."""
    dist = np.linalg.norm(points[:, None] - points[None], axis=2)
    a, b = np.unravel_index(dist.argmax(), dist.shape)
    rest = [i for i in range(len(points)) if i not in (a, b)]
    return points[[a, b] + rest]


def projector(g):
    return np.eye(3) - np.outer(g, g)


def smallest_mode(points, pairs):
    """Smallest eigenvalue of the follower bearing Laplacian, with its eigenvector."""
    n = len(points)
    lap = np.zeros((3 * n, 3 * n))
    for a, b in pairs:
        g = points[b] - points[a]
        p = projector(g / np.linalg.norm(g))
        i, j = 3 * a, 3 * b
        lap[i:i + 3, i:i + 3] += p
        lap[j:j + 3, j:j + 3] += p
        lap[i:i + 3, j:j + 3] -= p
        lap[j:j + 3, i:i + 3] -= p
    values, vectors = np.linalg.eigh(lap[6:, 6:])
    return values[0], vectors[:, 0]


def grow_graph(points, spacing, neighbors=3, min_mode=0.06):
    """Nearest neighbors, plus the short edges that raise the weakest mode the most."""
    n = len(points)
    dist = np.linalg.norm(points[:, None] - points[None], axis=2)
    pairs = set()
    for a in range(n):
        for b in np.argsort(dist[a])[1:neighbors + 1]:
            pairs.add((min(a, int(b)), max(a, int(b))))
    candidates = [(a, b) for a in range(n) for b in range(a + 1, n) if dist[a, b] < 6.0 * spacing]
    gain = np.array([projector((points[b] - points[a]) / dist[a, b]) for a, b in candidates])
    while True:
        mode, vector = smallest_mode(points, pairs)
        if mode >= min_mode:
            break
        moved = np.concatenate((np.zeros(6), vector)).reshape(-1, 3)
        delta = np.array([moved[a] - moved[b] for a, b in candidates])
        score = np.einsum('ni,nij,nj->n', delta, gain, delta)
        for k in np.argsort(-score):
            if candidates[k] not in pairs:
                pairs.add(candidates[k])
                break
        else:
            raise ValueError('The bearings cannot fix this formation: use fewer drones or letters.')
    edges = []
    for a, b in sorted(pairs):
        edges += [(a, b), (b, a)]
    return edges


def ground_pads(count, center):
    """Grid of pads on the ground around a point."""
    side = int(np.ceil(np.sqrt(count)))
    grid = [(i, j) for i in range(side) for j in range(side)]
    grid.sort(key=lambda p: (p[0] - (side - 1) / 2) ** 2 + (p[1] - (side - 1) / 2) ** 2)
    pads = np.zeros((count, 3))
    pads[:, :2] = (np.array(grid[:count]) - (side - 1) / 2) * PAD_SPACING + center[:2]
    return pads


def assign(sources, goals):
    """Goals reordered for the sources. With the smallest sum of squared distances,
    drones on straight lines that start and arrive together stay apart."""
    cost = ((sources[:, None] - goals[None]) ** 2).sum(axis=2)
    _, chosen = linear_sum_assignment(cost)
    return goals[chosen]


def paper_scenario(scale):
    start = PAPER_START * scale
    pads = start.copy()
    pads[:, 2] = 0.0
    edges = [(a - 1, b - 1) for a, b in PAPER_EDGES]
    return Scenario(PAPER_TARGET * scale, start, pads, edges, 0.0, 8.0)


def shape_scenario(points, spacing, seed=1):
    target = leaders_first(points)
    rng = np.random.default_rng(seed)
    start = target + rng.uniform(-START_NOISE, START_NOISE, target.shape)
    start[:2] = target[:2]
    pads = assign(start, ground_pads(len(target), target.mean(axis=0)))
    lifted = pads + np.array([0.0, 0.0, HOP_HEIGHT])
    travel = max(8.0, float(np.linalg.norm(start - lifted, axis=1).max()) / MAX_TAKEOFF_SPEED)
    return Scenario(target, start, pads, grow_graph(target, spacing), CLIMB_SEC, travel)


def landing_goals(pos, pads):
    """The pads, moved under the formation and given to the closest drones."""
    goals = pads + np.array([*(pos[:, :2].mean(axis=0) - pads[:, :2].mean(axis=0)), 0.0])
    return assign(pos, goals)
