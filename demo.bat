call "c:\opt\ros\melodic\x64\setup.bat"
call "c:\build2019_ws\devel_isolated\setup.bat"
roslaunch k4a_arm_support k4a_demo.launch robot_ip:=169.254.164.155 -v