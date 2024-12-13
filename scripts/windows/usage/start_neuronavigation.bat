@echo off

REM This script is used to start neuronavigation on the Windows computer in the lab in Aalto. Its contents are
REM bound to change; for instance, we currently have to run neuronavigation outside Docker to be able to use
REM Optitrack motion tracking. Once Optitrack works with Docker, the contents of this script could be changed to
REM something like:
REM
REM docker-compose up neuronavigation

call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64

call C:\dev\ros2_iron\local_setup.bat

cd %MTMS_ROOT%\ros2_ws

REM Note that neuronavigation is also rebuilt when running this script.

colcon build --packages-select neuronavigation

call install\local_setup.bat

REM E-field is disabled for now, enable when it works in production.
ros2 run neuronavigation start --ros-args -p electric_field_enable:=false -p robot_enable:=true

REM Ask for a keypress before closing the terminal window so that potential errors are shown to the user.
pause
