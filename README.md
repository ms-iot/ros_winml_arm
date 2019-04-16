# ros_winml_arm
Robot Arm Demo with WinML


# How to build this demo

```batch
:: open a ROS build window.
md c:\demo_ws
cd c:\demo_ws

:: download demo.rosinstall to c:\demo_ws
wstool init src demo.rosinstall

:: install the system dependencies and Python modules.
rosdep update
rosdep install --from-paths src --ignore-src -r -y

:: now build everything.
catkin_make_isolated --use-ninja --only-pkg-with-deps ur3_k4a-moveit_config --merge
```

```batch
:: additional required modules for rosbridge
pip install ptvsd

:: use tornado 4.x because newer version is not yet supported by rosbridge
pip uninstall tornado
pip install -U tornado==4.5.3
```

# How to run this demo

> Important Notes: `ur_driver.exe` needs to be allowed in the firewall because the UR3 driver modal requires the robot to talk back to the host machine over TCP socket.

```batch
:: in the same ROS build window.

devel_isolated\setup.bat
roslaunch k4a_arm_support k4a_demo.launch robot_ip:=169.254.164.155 -v
```

# How to build this demo (UWPApp)

```batch
:: Install Visual Studio 2017.
choco install visualstudio2017community

:: Install Windows 10 SDK 1809.
choco install windows-sdk-10-version-1809-all
```

1. Open ".\UWPApp\embedded_world.sln"
2. Now you can build & debug the App.

# Useful commands

```batch
:: launch moveit planning for UR3
roslaunch ur3_k4a-moveit_config UR3_with_k4a_execution.launch robot_ip:=169.254.164.155 -v

:: pickup engine block
rostopic pub --once /goto std_msgs/Int32 1
```