# duckie-killer
5LIA0 project

## Setup
Please download and place the repository in `~/GIT/duckie-killer/`. If located elsewhere, it is advisable to move the folder accordingly.

### ROS2 Installation
Ensure you are using Ubuntu 22.04 and have ROS2 installed. The easiest installation method is via the script available at the [Rover Robotics GitHub repository](https://github.com/RoverRobotics/rover_install_scripts_ros2/blob/main/ros2-humble-main.sh).

For rover setups, execute the `setup_rover` script from the same repository. Accept only the first and third options (download rover repository & `udev` rules).

Install necessary ROS dependencies by navigating to this repo's `src` directory and executing `rosdep install --from-paths . --ignore-src -r -y`.

Upon initial download of the repository, execute `colcon build` in `~/GIT/duckie-killer/` to build the system and generate the required setup scripts. 

### .bashrc and Group Setup
Add `source ~/GIT/duckie-killer/install/setup.bash` to your `.bashrc` file. If you prefer to avoid the impact on terminal initialization time, you can omit this and instead execute `source sources.sh` in each new window used for ROS2, enhancing terminal responsiveness.

To provide the program with access to GPS devices, add your user to the necessary groups with the following commands:

```
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER
sudo usermod -a -G video $USER
```

Log out and log back in to apply these changes. Confirm with `groups` command.

## Building and Running
Use `colcon build` in the workspace directory to build the project. then run `source sources.sh` to update the executables. 


