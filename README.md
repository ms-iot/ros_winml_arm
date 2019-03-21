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
catkin_make_isolated --use-ninja --only-pkg-with-deps k4a_arm_support --merge
```

```batch
:: additional required modules for rosbridge
pip install ptvsd
```

# How to run this demo

```batch
:: in the same ROS build window.

devel_isolated\setup.bat
roslaunch k4a_arm_support k4a_demo.launch
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