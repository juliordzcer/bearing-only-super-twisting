# Bearing-Only Super-Twisting Formation Control: Gazebo Simulation

Gazebo simulation based on the paper *Bearing-Only Super-Twisting Control for Multi-Quadrotor Formations* (CCE 2025). Ten ESP-drones fly in a two-level formation. Two leaders follow a time-varying velocity that the followers do not know, and the other eight keep their place using only the bearings to their neighbors. The followers can use the Super-Twisting law of the paper (`sta`) or the baseline proportional-derivative law (`pd`).

![Trajectory of the formation](media/trajectory.gif)

This repository is based on the simulation of the paper, but it uses another drone and other code. See [Differences from the paper](#differences-from-the-paper).

## What you need

- A computer with **Ubuntu 24.04**.
- About 5 GB of free disk space. No special GPU is needed, because the drones have no cameras here.

You do not need to know ROS or Python to run it, and ROS is not used. Copy each block into a terminal (`Ctrl+Alt+T` opens one) and press Enter.

## Installation

**1. Download the repository**

```bash
sudo apt update && sudo apt install -y git
cd ~
git clone https://github.com/juliordzcer/cce2025-bearing-only-sta.git
cd cce2025-bearing-only-sta
```

**2. Install Gazebo and the other dependencies**

This asks for your password and takes a few minutes.

```bash
./install_deps.sh
```

**3. Build the drone plugins**

```bash
cmake -S . -B build
cmake --build build -j4
```

It should end with `Built target` lines and no errors.

## Running the simulation

From the repository folder, choose a controller:

```bash
python3 scripts/fly_formation.py --controller sta
```

Gazebo and the control panel open after a few seconds. In the Gazebo window, press the view button or scroll to find the drones. Then:

1. Press **Take Off** and wait until all the drones hover at the same height, about 11 s.
2. Press **Automatic Mode**. The formation starts to take shape and move, and the panel shows the bearing error.
3. Press **Land** to finish.

**Emergency Stop** turns the motors off, so the drones fall. Close the panel window to end the simulation.

### Options

| Option | What it does |
|---|---|
| `--controller` | `sta` or `pd` |
| `--scale` | Size of the formation and speeds. `1.0` is the size of the paper, and the default is `0.5` |
| `--headless` | Run Gazebo without its window |
| `--auto` | Fly without the panel: take off, fly, save the data and exit |
| `--duration` | Seconds of automatic mode with `--auto` |
| `--output` | Folder for the flight data with `--auto` |

## Reproducing the results

```bash
python3 scripts/fly_formation.py --controller sta --auto --headless --output results
python3 scripts/fly_formation.py --controller pd --auto --headless --output results
```

Each one takes about 75 s, prints the minimum, median and maximum of the bearing error, and saves the flight as a CSV.

## Results

The metric is the one in the paper, measured after the first 30 s of automatic mode. Values are from 2 flights of each controller at scale 0.5, and the two flights differ by less than 0.002.

| Controller | Minimum | Median | Maximum | Smallest distance between drones [m] |
|---|---|---|---|---|
| PD | 0.0061 | 0.0205 | 0.0342 | 0.41 |
| STA | 0.0183 | 0.0348 | 0.0530 | 0.20 |

These do not reproduce the paper, where STA has the lower error (median 0.0036 against 0.0081). Here both controllers were tuned for this drone, and the PD has the lower error. The velocity loop of the drone firmware is between these laws and the motors, and it does not exist in the same form on the Bebop 2 model of the paper.

## Differences from the paper

| | Paper | This repository |
|---|---|---|
| Drone | Parrot Bebop 2 | ESP-drone model, much smaller |
| Formation and speeds | As in the paper | Multiplied by 0.5 by default |
| Measurement | Bearings from the simulated poses | The same |
| Low-level control | Velocity interface of the model | Position, velocity and attitude loops of the drone firmware |
| Bearing rate | From the simulated velocities | From the poses, numerically differentiated and filtered |
| Gains | Table of the paper | Tuned in flight for this drone and scale |
| Simulator | Gazebo Ionic and ROS Rolling | Gazebo Harmonic, without ROS |

## TODO

- Add the Parrot Bebop 2 version, to compare directly with the paper.
- Add larger formations with the same strategy, such as a pyramid with many drones or figures like letters, to show that it scales to many more agents.

## Known limitations

- **Flights at the same time.** Simulations on one computer interfere with each other, and drones can stay on the ground. To run several at once, give each its own `GZ_PARTITION`, `GZ_DISCOVERY_MSG_PORT` and `GZ_DISCOVERY_SRV_PORT`.
- **Windows.** The panel was tested on a virtual display, and flights were tested with Gazebo in headless mode.

## Common problems

**`Gazebo did not start. Check that the plugins were built.`**: run the two commands of step 3 of the installation, from the repository folder.

**`No module named 'gz'`**: run `./install_deps.sh` again.

**The drones stay on the ground**: close everything, check that no earlier simulation is still running with `pgrep -a ruby`, and start again.

## Repository layout

| Folder | Content |
|---|---|
| `plugins` | Motor model, position controller and attitude controller for Gazebo |
| `models/espdrone` | ESP-drone model |
| `worlds` | Gazebo world |
| `scripts` | Bearing-only laws, flight and control panel |

## How to cite

If you use this code, please cite both the paper and this repository.

```bibtex
@inproceedings{lopezruiz2025bearing,
  author    = {López Ruiz, Frida G. and Rodríguez, Julio César and Becerra, Héctor M. and Ruiz León, Javier},
  title     = {Bearing-Only Super-Twisting Control for Multi-Quadrotor Formations},
  booktitle = {2025 22nd International Conference on Electrical Engineering, Computing Science and Automatic Control (CCE)},
  pages     = {1--6},
  year      = {2025},
  url       = {https://ieeexplore.ieee.org/document/11272041}
}
```

```bibtex
@software{rodriguez2026cce2025sim,
  author = {Rodriguez, Julio Cesar},
  title  = {CCE 2025 Bearing-Only Super-Twisting Simulation},
  year   = {2026},
  url    = {https://github.com/juliordzcer/cce2025-bearing-only-sta}
}
```

## License

The code is under the [Apache License 2.0](LICENSE). Keep the [NOTICE](NOTICE) file in copies and derived works.

The ESP-drone model in `models/espdrone` was designed by Julio Cesar Rodriguez, who also measured its parameters on the real drone. It is under the [Creative Commons Attribution 4.0](models/espdrone/LICENSE) license, so credit him if you use or adapt it.

Contact: Julio Cesar Rodriguez, contact@juliordz.com
