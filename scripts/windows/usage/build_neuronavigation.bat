@echo off

REM This script is used to build neuronavigation on the Windows computer in the lab in Aalto and Tubingen.
REM Note that the ROS workspace is not re-built; for that, use build_ros_workspace.bat.

call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64

call C:\dev\ros2_iron\local_setup.bat

cd %MTMS_ROOT%\ros2_ws

colcon build --packages-select neuronavigation

REM Ask for a keypress before closing the terminal window so that potential errors are shown to the user.
pause
