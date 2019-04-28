call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars64.bat"
call "c:\opt\ros\melodic\x64\setup.bat"
pushd c:\build2019_ws
catkin_make_isolated --use-ninja --only-pkg-with-deps ur3_k4a-moveit_config --merge