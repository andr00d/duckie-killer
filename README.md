# duckie-killer
5LIA0 project

## Setup
Please download and place the repository in `~/GIT/duckie-killer/`. If located elsewhere, it is advisable to move the folder accordingly.

### ROS2 Installation
Ensure you are using Ubuntu 22.04 and have ROS2 installed. The easiest installation method is via the two script available at the [Rover Robotics GitHub repository](https://github.com/RoverRobotics/rover_install_scripts_ros2).

to setup the workspace, clone the repo and first run the `ros2-humble-main.sh` script, this will setup the entirety of ros2 for you. 

For the rover setup, execute the `setup_rover,sh` script from the same repository. Accept only the first and third options (download rover repository & `udev` rules).

Install necessary ROS dependencies by navigating to this repo's `src` directory and executing `rosdep install --from-paths . --ignore-src -r -y`.

furthermore, run the following command to install depenencies necessary for building:  
```sudo apt-get install ros-humble-tf-transformations```   
``` pip3 install transforms3d```   

Upon initial download of the repository, execute `colcon build` in `~/GIT/duckie-killer/` to build the system and generate the required setup scripts. 

### .bashrc and Group Setup
Add `source ~/GIT/duckie-killer/install/setup.bash` to your `.bashrc` file. If you prefer to avoid the impact on terminal initialization time, you can omit this and instead execute `source sources.sh` in each new window used for ROS2, enhancing terminal responsiveness.

To provide the program with access to GPS devices, add your user to the necessary groups with the following commands:

## Building and Running
Use `colcon build` in the workspace directory to build the project. then run `source sources.sh` to update the executables. 

one usefull command to run when debugging is:  
`colcon build && source sources.sh && ros2 launch navigation gesture_launch.py`