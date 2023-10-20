ros_task_manager for ROS2 Humble, Ubunru 22.04
==============================================

Generic Task Manager for ROS. 

The complete documentation and details are available on:
https://hal.archives-ouvertes.fr/hal-01435823

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
If everything works, you should see a turtle drawing a square.

![image](https://github.com/JunghwanRo/ros_task_manager/assets/112362005/1edb323d-eab0-433b-b9ea-90c5286b59f6)

## Tutorial 

The easiest way to start is the task_manager_turtlesim under src.
Copy the task_manager_turtlesim to your workspace src folder, and change the name as you want. 












