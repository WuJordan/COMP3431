# Project Overview

This package controls TurtleBot3 wall-following, marker detection, and autonomous navigation using Nav2 and Cartographer. It also includes tools for generating a map and recording marker locations for later navigation.
## File Architecture

**`src/wall_follower/launch/`**  
Launch files for the wall_follower package.  
Starts:

- `see_marker.py`
- `point_transformer.py`
- `wall_follower.cpp`

**`src/wall_follower/scripts/`**  
Helper Python scripts.

- `hsv.py`: colour picker utility used to determine HSV ranges for colour-masking calibration.

**`src/wall_follower/map/`**  
Contains `map.yaml` and `map.pgm` generated when saving a Cartographer map.

**`src/wall_follower/params/`**  
Nav2 configuration files (e.g., `waffle_pi.yaml`) loaded by the Nav2 stack.
## Build Instructions

From inside **`~/turtlebot3_ws`**:

`colcon build --symlink-install`

Source your workspace if not already:

`source install/setup.bash`

## Run Instructions
### **Step 1  Start the robot**

SSH into the TurtleBot3 and run the following command to initialise the robot:

`bringup` 

When the lidar begins to spin, and the ssh terminal indicates the robot is running, run the start camera command to see th camera feed.

`start_camera`

### **Step 2  Generate map and marker positions**

Start Cartographer:

`ros2 launch turtlebot3_cartographer cartographer.launch.py`

Start wall_follower:

`ros2 launch wall_follower wall_follower.launch.py`

**Important:** `markers.csv` is only saved when you **Ctrl+C** the wall_follower terminal.  
Do this after exploring the environment and detecting the markers.

### **Step 3  Save the map**

Run from the robot:

`ros2 run nav2_map_server map_saver_cli -f /home/pi/turtlebot3_ws/src/wall_follower/map/map`

This produces both `map.yaml` and `map.pgm`.


### **Step 4  Run navigation with imported map**

Start the marker navigation node:

`ros2 run wall_follower navigate_marker.py`

Start Nav2 with the saved map and configuration:

`ros2 launch turtlebot3_navigation2 navigation2.launch.py \     map:=/home/pi/turtlebot3_ws/src/wall_follower/map/map.yaml \     params_file:=/home/pi/turtlebot3_ws/src/wall_follower/params/waffle_pi.yaml`

**Note:**  
`markers.csv` must be located in the **workspace root**