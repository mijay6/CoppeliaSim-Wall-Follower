# CoppeliaSim Wall Follower

[![Language: Python](https://img.shields.io/badge/language-Python-blue.svg)](https://en.wikipedia.org/wiki/Python_(programming_language))
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![University: UPT](https://img.shields.io/badge/University-Politehnica%20Timisoara-red.svg)](https://www.upt.ro/)
[![Status: Academic Project](https://img.shields.io/badge/Status-Academic%20Project-success.svg)](https://github.com/mijay6/CoppeliaSim-Wall-Follower)
[![Version](https://img.shields.io/badge/Version-1.0.0-brightgreen.svg)](https://github.com/mijay6/CoppeliaSim-Wall-Follower/releases)

## Description

This repository contains a complete setup for a **wall‑follower** robot simulation using **CoppeliaSim** and a Python‑based controller.  The project includes a ready‑made scene with a Pioneer P3DX mobile robot, a configurable PID controller implemented in Python, and auxiliary scripts to connect to the simulator using a Remote API.  The goal of this project is to provide a clear, educational example of how to build and tune a wall‑following-PID algorithm.

## Dependencies

This project relies on several Python packages and system libraries to communicate with CoppeliaSim and process sensor data:

- **CBOR** – Used to serialise and deserialise messages between Python and CoppeliaSim.  Install it via apt or pip:
  ```bash
  sudo apt install python3-cbor      
  # or, using pip
  python3 -m pip install cbor
  ```
- **ZeroMQ (pyzmq)** – Provides asynchronous messaging and sockets for the Remote API:
  ```bash
  sudo apt install python3-zmq       
  # or, using pip
  python3 -m pip install pyzmq
  ```
- **NumPy** – Numerical library used for handling sensor arrays:
  ```bash
  python3 -m pip install numpy
  ```
- **OpenCV (optional)** – Required only if you plan to use the camera from CoppeliaSim:
  ```bash
  python3 -m pip install opencv-python
  ```

If CoppeliaSim fails to start due to a missing **Qt platform plugin** (error mentioning `xcb`), install the required system libraries as follows:

```bash
sudo apt-get update && sudo apt-get install -y \
  libxcb-xinerama0 libxcb-icccm4 libxcb-image0 \
  libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 \
  libxcb-shape0 libxcb-xfixes0 libxcb-xkb1 libxkbcommon-x11-0
```

## How to Use

1. **Configure the environment** – Before running any Python scripts, source the configuration script to set up the correct `PYTHONPATH`:
   ```bash
   source config.bash
   ```
   This step must be repeated each time you open a new terminal so that Python can find the Remote API client.

   Verify if the CoppeliaSim API client library (ZeroMQ) is installed correctly.
   ```bash
   python3 -c "from coppeliasim_zmqremoteapi_client import RemoteAPIClient; print('OK')"
   ```

3. **Open the simulation** – Launch **CoppeliaSim** and open the scene file located in `scenes/wallFollower.ttt`.  The scene contains the Pioneer P3DX robot, the floor and walls for wall‑following, and a small child script that draws the robot’s trajectory for visualisation.

4. **Run the controller** – From the root of the repository, execute the Python control script:
   ```bash
   python3 wall_follower_pid.py
   ```
   The script connects to the running simulator, starts the simulation, reads the sonar sensors and sends wheel commands to perform wall following using a configurable PID controller.  Tune the gains in `Config` within `wall_follower_pid.py` to change the behaviour of the robot.

## Documentation

- **CoppeliaSim** – Site: <https://www.coppeliarobotics.com/>  •  Manual: <https://manual.coppeliarobotics.com/index.html>
- **Robot Documentation** – <https://www.cyberbotics.com/doc/guide/pioneer-3dx?version=R2021a>
- **Reference Projects** – The implementation in this repository is inspired by and adapted from:
  - <https://github.com/jdlope/robotica/tree/master>
  - <https://github.com/mijay6/Wall-Following-Practica-Robotica>

## Video Tutorial

- <https://youtu.be/WFJHBVnnXt8>

## Author

Dobra Mihai

Politehnica University of Timișoara  
Faculty of Automation and Computer Science  
Robotics Elements  
Academic Year 2025–2026

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct and the process for submitting pull requests.
