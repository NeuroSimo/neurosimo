# MATLAB API for mTMS software

## Using API

### Getting started

Create an instance of MTMSApi class and start the mTMS device by running:

```
api = MTMSApi();
api.start_device();
```

See `example.m` for a more thorough example on how to use the API.

## Troubleshooting

### Problem: service calls do not finish

- MATLAB ROS Toolbox uses a different ROS Middleware Implementation (RMW) underneath than, for example, Galactic. Read the following discussion for more information:
https://se.mathworks.com/matlabcentral/answers/1777145-ros-toolbox-stuck-on-service-call.
To fix this issue, run the following command:
`setenv("RMW_IMPLEMENTATION","rmw_cyclonedds_cpp")`.

### ros2genmsg error: `No packages with '.msg' files found under [path to mTMS repository]/ros2_ws/src/interfaces/fpga_interfaces. Each message package directory must contain a directory named 'msg' that then contains '.msg' files.`
- Ensure that you are running ros2genmsg with the correct path; the path parameter should be, e.g., '/home/mtms/mtms/ros2_ws/src/interfaces' instead of '/home/mtms/mtms/ros2_ws/src/interfaces/fpga_interfaces'.
- Check that you do not have subdirectories inside any of the `msg` directories (however, there can be subdirectories inside `srv` directories).
- Check also that the paths to msg and srv files are correctly defined in CMakeLists.
