# 🚗 Trace Car — ROS2 Autonomous Line-Following Simulation

A ROS2-based autonomous trace-car simulation built in Gazebo, featuring a custom race track and vehicle model.
## 📺 Demo
[![Trace Car Demo](https://img.youtube.com/vi/a4iX7MZsy_s/0.jpg)](https://youtu.be/a4iX7MZsy_s)

<table>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/c854146b-1831-4464-9839-9ac7ea32f01d" width="400"/>
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/9023b229-7792-489a-9ee2-273ca64052ec" width="400"/>
    </td>
  </tr>

  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/be513d4b-58ba-449d-951a-22ada518f548" width="400"/>
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/3bc72f6e-9ed5-4e6a-a96e-3f8de64ada06" width="400"/>
    </td>
  </tr>
</table>

## Introduction
`trace_car` is a ROS 2 + Gazebo line-tracing car project built with `colcon` and `CMake`, and implemented with both C++ and Python. It uses CCD-style perception, a controller state machine, and a Gazebo camera/control plugin to simulate an autonomous car following a road map in a closed track.

The project is centered on these goals:

- build the car model from CAD/Fusion360 assets and convert meshes into Gazebo-friendly formats
- build a 9 m x 9 m road/ground map and surround it with walls
- spawn the model in Gazebo, attach the camera and ros2_control plugins, and run the controller stack
- detect road position and road type from CCD images using NumPy-based image processing
- publish steering and wheel commands to keep the car on track
- provide a Tkinter GUI for simulation control, parameter tuning, and live status monitoring

## Tech Stack

- ROS 2
- Gazebo
- `colcon` / `CMake`
- NumPy
- OpenCV
- Tkinter
- Qt / Gazebo plugin interface



## Project Workflow

This repository follows the same development flow used in the codebase:

1. Install the ROS 2, Gazebo, controller, and Python dependencies with [scripts/install_deps.sh](scripts/install_deps.sh).
2. Build the model from CAD/Fusion360, export to `obj`, then convert to `dae` for Gazebo.
3. Build the road and ground texture/mesh assets for the 9 m x 9 m simulation area.
4. Create the ROS 2 package `trace_car` and wire the model, world, and messages into the build.
5. Build the world configuration.
	- Add the road map into Gazebo.
	- Add surrounding walls.
6. Build the car model configuration.
	- `body`
	- `wheel_L_back`
	- `wheel_R_back`
	- `steer_axis_L`
	- `steer_axis_R`
	- `wheel_L_front`
	- `wheel_R_front`
	- `ccd_close`
	- `ccd_far`
7. Build the launch files.
	- `gzserver`
	- `gzclient`
	- car model publisher
	- car model spawn
8. Add the Gazebo plugins.
	- `libgazebo_ros_camera.so`
	- `libgazebo_ros2_control.so`
9. Add `controller_manager` launch steps.
	- `joint_state_broadcaster`
	- `wheel_velocity_controller`
	- `steer_position_controller`
10. Implement `ccd_node.py` / `ccd_node.cpp`.
	- startup warm-up loop
	- subscribe to CCD camera images
	- spatial median filter
	- temporal median filter
	- local smoothing and edge detection
	- enumerate edges, build light windows, and choose the correct lane window
	- compute road center and center error
	- debounce road type detection
	- publish `road_type` and `center_error` to the controller node
11. Implement `controller_node.py` / `controller_node.cpp`.
	 - startup warm-up loop
	 - subscribe to perception data
	 - PD steering control from center error
	 - speed target selection from road type
	 - state machine for cross/stop locking
	 - cross unlock after odometer distance
	 - cross center-error decay for lane estimation
	 - steering dynamics limiting and Ackermann steering calculation
	 - subscribe to Gazebo joint states
	 - record odometer during cross and stop
	 - speed dynamics limiting and rear-wheel differential speed calculation
	 - publish steering and wheel commands back to Gazebo
12. Build the Tkinter GUI.
	 - CCD image monitor: image, left edge, right edge, center, road width, average brightness
	 - car information monitor: steer, speed, road type
	 - simulation control: start, pause, reset
	 - runtime parameter tuning via `declare_parameter` + sliders
	 - show and hide raw CCD images even under strong noise
13. Add user camera switching in Gazebo.
	 - self-built plugin reads `camera_view_cmd`
	 - reads `car_info` to get current steering state
	 - applies frame transformation and pose composition
	 - supports global static view, global track view, first-person view, and third-person view
14. Convert the Python implementation to C++, complete the `CMakeLists.txt`, and build the whole project.
15. Add `ccd_node`, `controller_node`, and GUI monitor nodes to the launch file.

## Current Package Layout

- `launch/`: Gazebo and robot launch files
- `models/`: robot and mesh assets
- `worlds/`: track/world definitions
- `config/`: controller configuration
- `msg/`: custom ROS 2 messages
- `src/`: C++ nodes and plugin code
- `trace_car/`: Python nodes and logic
- `rviz/`: RViz configuration
- `scripts/`: helper scripts

## Main Runtime Pieces

- `launch/car_sim.launch.py`: starts the full simulation stack
- `launch/car_model.launch.py`: publishes the robot description and spawns the model
- `trace_car/ccd_node.py` and `src/ccd_node.cpp`: CCD perception pipeline
- `trace_car/controller_node.py` and `src/controller_node.cpp`: steering and wheel control pipeline
- `trace_car/monitor_gui.py`: Tkinter GUI for simulation control and live monitoring
- `src/camera_view_plugin.cpp`: custom Gazebo camera plugin

## Install Dependencies

You can install the project dependencies with the helper script in [scripts/install_deps.sh](scripts/install_deps.sh).

From the package root:

```bash
chmod +x scripts/install_deps.sh
./scripts/install_deps.sh
```

The script installs the ROS 2, Gazebo, controller, and Python packages used by the project, then prints the workspace build command for the next step.

## Building

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

If this is a new shell, source your ROS 2 installation first, then source the workspace overlay.

## Running

Launch the full simulation:

```bash
ros2 launch trace_car car_sim.launch.py
```

This brings up:

- Gazebo server
- Gazebo client
- robot description publisher and model spawn
- `ccd_node`
- `controller_node`
- GUI monitor

## Runtime Topics

The main nodes communicate through these topics:

- `/ccd/ccd_close/image_raw` and `/ccd/ccd_far/image_raw`: CCD camera images
- `/perception`: road type and center error
- `/joint_states`: wheel joint feedback from Gazebo
- `/wheel_velocity_controller/commands`: rear wheel speed commands
- `/steer_position_controller/commands`: steering commands
- `/car_info`: controller state for the GUI
- `/ccd_detection`: CCD monitor data for the GUI
- `/gui_control`: start, pause, and reset commands
- `/camera_view_cmd`: camera view switching commands

## CCD Pipeline

The CCD node is designed around a stable warm-up and a two-stage filter pipeline:

- wait for a fixed number of frames before publishing control data
- apply a spatial median filter to suppress pixel noise
- apply a temporal median filter to stabilize frame-to-frame fluctuations
- smooth the detected light window before doing edge detection
- detect left and right edges from averaged brightness differences
- choose the best window around the road center
- compute road center and center error
- debounce road type classification before publishing it

## Controller Pipeline

The controller node turns perception into motion with a small state machine:

- use center error for steering PD control
- use road type for target speed selection
- lock cross and stop decisions when they are detected
- release cross lock after the odometer passes the cross distance
- decay the stored cross center error during the cross section to estimate lane position
- limit steering change rate and speed change rate
- compute Ackermann steering and rear wheel speed split
- publish commands to the Gazebo controllers

## GUI

The Tkinter GUI is used for interactive tuning and monitoring:

- CCD image monitor with edges, center, road width, and average brightness
- car information monitor with steering, speed, and road type
- simulation controls for start, pause, and reset
- live parameter adjustment through sliders backed by ROS 2 parameters
- raw CCD image display toggle for noisy input debugging

## Camera Views

The custom Gazebo camera plugin supports multiple viewpoints:

- `GLOBAL static view`: set once in pre-render and keep the camera at a fixed pose
- `GLOBAL track view`: follow the car position while keeping a preset camera rotation
- `FIRST PERSON view`: compose the car pose with a relative pose
- `THIRD PERSON view`: combine car pose, steering yaw, and a relative offset, with low-pass filtering to reduce jitter

## Notes

- The package is tuned around the included CCD image width and track geometry.
- If you change the mesh names, joint names, or topic names, update the launch files, model file, and controller code together.
- The current codebase contains both Python and C++ versions of the perception/controller stack; the C++ path is already wired into `CMakeLists.txt`.

## Troubleshooting

- If Gazebo cannot find the plugin, make sure the workspace has been built and sourced.
- If the car does not move, confirm that `controller_manager`, `joint_state_broadcaster`, and the wheel/steer controllers are running.
- If the CCD pipeline is noisy, lower the raw-image threshold values or adjust the GUI parameters.
