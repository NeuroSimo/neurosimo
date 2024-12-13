# Real-time pipeline
![real-time pipeline](pipeline_nodes.svg)

TODO: This document is very much outdated by Nov 2023, please update!

The real-time pipeline consists of 3 main nodes: EEG bridge, EEG processor, and mTMS device bridge.
1. EEG bridge receives data from the EEG device and serializes it into a format other ROS nodes understand. The data includes EEG samples and triggers.
2. EEG processor receives the EEG samples, passes them to a hot-swappable component which outputs mTMS events, and forwards the events to either stimulus presenter (stimuli events) or mTMS device bridge (mTMS device events).
3. mTMS device bridge receives the mTMS device events, serializes them, and writes them to the mTMS device.

In addition, the pipeline supports two additional nodes: EEG preprocessor and Stimulus presenter. They both follow the same hot-swappable design as EEG processor.
1. EEG preprocessor receives raw EEG samples as input, passes them to a hot-swappable algorithm which outputs cleaned EEG samples, and then sends these cleaned EEG samples to EEG processor.
2. Stimulus presenters receives stimuli events, passes them to a hot-swappable algorithm which outputs mTMS device evens, and then sends these mTMS device events to the mTMS device bridge.

So, in total, the real-time pipeline can consist of 3 stages + 2 bridges. It is not necessary to use all 3 stages, only EEG processor is necessary. If not using EEG preprocessor, be sure to set the value of `preprocess` to false when starting EEG processor. Otherwise it subscribes to cleaned EEG data and won't receive the raw EEG data.


If only simulating, the situation becomes simpler. EEG bridge is replaced with EEG simulator, and mTMS device bridge is removed altogether. EEG simulator is a ROS node which takes as an input a CSV file, which contains an EEG sample on each row.


## Installation for simulation environment
1. Git clone mtms repository and git checkout to eeg-processor-launch-configurations
2. Run the installation script `install_ros.sh` from the mtms/scripts directory
3. Source ros
4. Go to mtms/ros2_ws directory
5. `colcon build --packages-select eeg_interfaces event_interfaces mtms_device_interfaces targeting_interfaces`
6. `colcon build --packages-select eeg_processor --cmake-args -DCMAKE_BUILD_TYPE=Release`
7. `colcon build --packages-select eeg_simulator`

## Installation for real environment
There is no need to install the real environment for development, as it cannot be used on any other computer than the mTMS lab computer. However, if you are installing a new computer for the lab, see `installation.md` for instructions.

## Sourcing ROS
After you have installed ROS, you need to also source ROS each time you want to use it. Otherwise, ROS is not on the path, and it cannot be used. Moreover, you also need to source our ROS nodes. It might be useful to either alias these commands, or to add them to .bashrc.
1. `source /opt/ros/iron/setup.bash`
2. `source /path/to/mtms/ros2_ws/install/local_setup.bash`


## Simulated environment
In the simulated environment, it is sufficient to launch EEG simulator and the wanted pipeline stages.

## Real environment
In the real environment, follow these steps.

### Set up
1. Connect a BNC cable to the sync port of the cabinet and to the trigger IN port A of the EEG device. In addition, if using triggers, connect another BNC cable from any trigger OUT port of the mTMS device to the trigger IN port B of the EEG device.
2. Connect the EEG deck laptop to the host machine via Ethernet cable. At Aalto, connect it to the switch that is resting on top of the mTMS PC.
3. Turn on NeurOne, amplifiers, battery, and the EEG deck laptop. Open NeurOne and select the desired protocol.
4. Turn on the cabinet from the back.

### Start software
5. Launch mTMS device bridge.
6. Launch EEG device bridge.
7. Start Web UI and rosbridge by running `docker-compose up front rosbridge` from the root of the mtms repository.
8. Launch EEG processor and other desired pipeline stage nodes.
9. Start measurement from the EEG deck laptop.
10.  On web UI, press 'Start device'. Wait until the device starts and then press 'Start session'. This starts the session and this is when EEG bridge starts publishing data.
11. When you are ready to end the session, press 'Stop session' and 'Stop device'.

## Launching nodes
### With Docker
With docker, running the nodes is simple. All except mTMS device bridge can be launched with Docker.

Simply run `docker-compose up <list of nodes>`, e.g. `docker-compose up eeg_bridge eeg_processor`.

The ROS launch arguments for these nodes are automatically read from .env file, so change the variables there to change the values of the passed arguments.


### Without Docker
Each of the ROS nodes needs to be started in its own terminal. On each new terminal, you need to source ROS and source the ROS nodes before you can launch the ROS node.

Fill in the launch arguments as needed. In particular, pay attention to the number of channels in EEG bridge. See `mtms/ros2_ws/src/eeg/eeg_processor/README.md` for details about the launch arguments regarding pipeline stages.

#### EEG simulator
`ros2 launch eeg_simulator eeg_simulator.launch.py log-level:=info data-file:=/path/to/mtms/ros2_ws/src/eeg/eeg_simulator/data/random_data.csv sampling-frequency:=5000 loop:=true`


#### EEG bridge
`ros2 launch eeg_bridge eeg_bridge.launch.py log-level:=info port:=50000 number-of-eeg-channels-amplifier-1:=32 number-of-emg-channels-amplifier-1:=0 number-of-eeg-channels-amplifier-2:=32 number-of-emg-channels-amplifier-2:=0`


#### EEG preprocessor
`ros2 launch eeg_processor eeg_preprocessor.launch.py log-level:=info preprocessor-type:=python preprocessor-script:=pipeline.python.preprocessor`


#### EEG processor
`ros2 launch eeg_processor eeg_processor.launch.py log-level:=info processor-type:=python processor-script:=pipeline.python.python_processor preprocess:=false`


#### Stimulus presenter
`ros2 launch eeg_processor stimulus_presenter.launch.py log-level:=info stimulus-presenter-type:=python stimulus-presenter-script:=pipeline.python.stimulus_presenter`


#### mTMS device bridge
`ros2 launch mtms_device_bridge mtms_device_bridge.launch.py log-level:=info safe-mode:=false`

#### Notes
- Source ROS on each new terminal before running nodes
- If using python as the type of the hot-swappable component, the pipeline stage node (e.g. EEG processor) must be run from the root of the mTMS repository.

