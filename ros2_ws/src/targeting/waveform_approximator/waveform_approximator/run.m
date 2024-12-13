% XXX: Registering the messages would fit better in ros_entrypoint.sh,
%   but running it separately there causes MATLAB to start twice, once
%   for the registering the messages and once for running the present script.
%
%   In Aalto, running the Docker container with network_mode: host causes
%   MATLAB to take around 2 minutes to start up, which is why we want to
%   avoid starting it twice.
ros2RegisterMessages('/home/matlab');

wrapper = WaveformApproximatorWrapper();

% An infinite loop to let the ROS node run in the background.
while true
    pause(1);
end
