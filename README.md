# ros_winml_arm
Robot Arm Demo with WinML


# How to build this demo

```
:: open a ROS build window.
md c:\demo_ws
cd c:\demo_ws

:: download demo.rosinstall to c:\demo_ws
wstool init src demo.rosinstall

:: now build everything.
catkin_make_isolated --use-ninja --only-pkg-with-deps k4a_arm_support --merge
```

# How to run this demo

```
:: in the same ROS build window.

devel_isolated\setup.bat
roslaunch k4a_arm_support k4a_demo.launch
```