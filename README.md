ros_task_manager for ROS2 Humble, Ubunru 22.04
==============================================

Generic Task Manager for ROS. 

The complete documentation is available on:
https://hal.archives-ouvertes.fr/hal-01435823

# Example
## Example
### Example
Example

# How to start
clone the repository 
```
git clone https://github.com/cedricpradalier/ros_task_manager.git
```
Change the branch
```
git checkout ros2-humble
```
After moving to your workspace directory, do Colcon Build. 
```
colcon build
```
## Confirm proper installation
Let's confirm the installation.
```
ros2 launch task_manager_turtlesim launch_all.launch
```
If you used --merge-install for your colcon build, 
```
ros2 launch task_manager_turtlesim launch_all_mi.launch
```
