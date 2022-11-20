# F1Tenth-AEB实验

## Source

https://github.com/JC0103?tab=repositories

## Dependeices for Running Locally
* ROS Melodic
* F1TENTH Simulator

## Basic Build Instructions

1. Clone this Package into catkin_ws/src.
2. Build the ROS package：catkin_make.
3. Configuration File：source devel/setup.bash.
4. Launch it：roslaunch huangyiwen_aeb AEB_simulation.launch.
5. Operating Step1：first press k to start the keyboard.
6. Operating Step2：press w to move the trolley forward.
7. Operating Step3：press b to start AEB.
8. Operating Step4：press a to change the trolley's forward direction.
9. Operating Step5：observe the emergency braking results of the console trolley.

## Problem and Suggested modifications

1. The collision during turning is still possible as the lidar sensing until TTC calculation still need a period of time. (We can slow down the speed during turning, or come out with higher TCC thersold during turning)
