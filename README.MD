# Fully Autonomous Dingo POC 

This repo is dedicated to work on a fully autonomous workflow for the Dingo-O in ROS2 inclusive of docking, NAV2/Slam Toolbox autonomy and then workflow management. 


## Basic use of the Clearpath NAV2 Demo

At the moment I have attached a configuration "robot.yaml" to use with the Clearpath NAV2 demo:

https://docs.clearpathrobotics.com/docs/ros/tutorials/navigation_demos/overview


If you are having issues with networking you can use the following command to generate a setup.bash file but it's also recommended to remove any ROS2 networking lines from your ~/.bashrc. 

```bash
ros2 run clearpath_generator_common generate_bash -s ~/FAD_POC
source ~/FAD_POC/setup.bash #or add to ~/bashrc
```