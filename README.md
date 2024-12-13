# NeuroSimo

## Overview
NeuroSimo is an open-source software platform designed for real-time EEG- and EMG-guided transcranial magnetic stimulation (TMS). It provides tools for precise closed-loop stimulation control, modular design, and seamless integration with existing TMS and EEG systems.

## License
NeuroSimo is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.

## Copyright and Permitted Use
All rights to the code in this repository are currently under review. Contributions were originally made under the mTMS project at Aalto University.

### Acknowledgments
This software was originally part of the mTMS project developed at Aalto University. Components of the original project were extracted and adapted to create NeuroSimo.

## Contributors
The following people have contributed to the development of NeuroSimo:

- Olli-Pekka Kahilakoski
- Kyösti Alkio
- Kim Valén
- Oula Siljamo
- Lisa Haxel
- Matilda Makkonen
- Tuomas P. Mutanen

The list of contributors may not be exhaustive. If you have contributed to the
development of NeuroSimo and would like to be added to the list, please contact
the authors.

## Installation

NeuroSimo is intended to be used with a dedicated computer with real-time Ubuntu installed.

NOTE: This section will be updated in December 2024 to reflect the latest installation instructions.

For now, steps for installing and configuring the system and the software can be found under
`Groups/SW Group/Computer installation` folder in ConnectToBrain Google Drive.

After those steps, re-booting the computer automatically starts the Docker containers that run NeuroSimo.

## Getting started

#### Creating a new project

A new project is created from the command line (ctrl+alt+t) by running the following command:

```
create <project_name>
```

After that, the project can be opened from the mTMS panel.

Here are some guidelines for choosing a project name:

- The project name should start with the researcher's name, e.g., "jarmo"
- The words in the name should be separated by hyphens, e.g., "jarmo-testing-pulses"

#### Before running the real-time pipeline

Open the mTMS panel and ensure that it reports that the mTMS device is powered on
and started. If not, first turn it on by pressing the power button on the device.
After that, press "Start device" on the front-page of the mTMS panel and wait for
the device to start.

Also ensure that a BNC cable is connected from the 'sync' port of the mTMS device
to 'Port A in' of the EEG/EMG device.

Check that the EEG/EMG device is connected to the real-time computer with an
Ethernet cable via the router.

The pulses created by the mTMS device are loud and can be sudden and unexpected;
ensure that all the people in the room are wearing hearing protection.

#### Running the real-time pipeline

On the mTMS panel, you can run the real-time pipeline by doing the following:

- Switch to "Pipeline" tab
- Select the project from the dropdown menu
- Select the preprocessor ("Example" is a good starting point)
- Preprocessor is always enabled, so you can leave it as is
- Select the decider ("Example" is a good starting point here as well)
- Enable the decider
- For now, you can leave "Presenter" disabled

**Note**: The example decider will perform a trial every 2 seconds. The
pulses associated with these example trials are relatively quiet (the pulse
intensity is set to 30 V/m at the maximum), but it is still recommended to
wear hearing protection while the pipeline is running.

After that, there are two options for streaming the EEG/EMG data to the pipeline:

1) Turn on streaming in the EEG/EMG device. The status bar in the top-right
corner should indicate that the device is connected and streaming. After that,
press "Start session" on the mTMS panel.

2) Using simulated data: in the bottom-right corner of the mTMS panel, there is
a dropdown menu for selecting the data set to use. Select "Random data, 1 kHz",
enable "Playback" and "Loop", and press "Start session". The "Loop" option
will restart the data set after it has been played through.

After the session has started, you can see what the pipeline is doing by following
its log messages on the command line. For instance, running

```
log preprocessor
```

will show and follow the log messages of the preprocessor. However, the default
preprocessor only passes the data through, so there is not much to see there.

Instead, you can run

```
log decider
```

to follow the log messages of the decider. The decider will print out the stimulation
decisions it makes based on the EEG/EMG data. Following the log will give an idea
of what the decider does: First, it precomputes the target locations. After the precomputation
is finished, it starts performing a trial every 2 seconds, switching between low-intensity,
medium-intensity, and paired-pulse trials.

Press "Stop session" on the mTMS panel to stop the pipeline from running.

#### Creating custom preprocessor and decider scripts

Once you have the real-time pipeline running with the example scripts, you can start
modifying the scripts to suit your needs.

The preprocessor scripts are located in `~/projects/<project_name>/preprocessor` and the decider
scripts are located in `~/projects/<project_name>/decider`. For instance, you can create a
copy of `example.py` with another name and use it as a starting point for your own script.

**Note**: The mTMS panel automatically updates the scripts in the `preprocessor` and
`decider` dropdown menus when the contents of the directories are modified. That is, after
copying `example.py` to another name, you can immediately start using the new script
by selecting it from the dropdown menu in the mTMS panel.

**Note**: When the real-time pipeline is running, the preprocessor and decider modules
automatically reload the script when it is modified. This means that you can modify the
scripts while the pipeline is running, and the changes will take effect immediately after
saving the script. However, beware that the state of the script is reset when it is reloaded.

#### Using custom data

The data sets used by EEG simulator are stored in `~/projects/<project_name>/eeg_simulator`.
Each data set is defined by two files: a CSV file containing the data and a JSON file
containing the metadata.

The naming of the files is arbitrary, but a good convention is to have the same name
for both files, with the CSV file ending in `.csv` and the JSON file ending in `.json`.

For instance, the data set `test_data` would be defined by the files `test_data.csv` and
`test_data.json`, with the JSON file looking like this:

```json
{
    "name": "Test data from experiment 1",
    "data_file": "test_data.csv",
    "event_file": "test_events.csv",
    "channels": {
        "eeg": 3,
        "emg": 1
    }
}
```

The data CSV file should contain the data in the following format:

```csv
0,0.1,0.2,0.3,1
0.001,0.11,0.21,0.31,2
0.002,0.12,0.22,0.32,3
...
```

The first column is the time in seconds, and the following columns are the EEG and EMG
channels in that order. The number of EEG and EMG channels should match the numbers
given in the JSON file.

The event CSV file is optional; if the field is undefined, the dataset won't contain any events. If defined,
the file should contain timestamps for the events, one timestamp per line, followed by the event type (an unsigned
integer). For example:

```csv
1.0,1
2.0,2
5.0,3
```

After creating a new data set, you can select it from the dropdown menu in the mTMS panel
where it should automatically appear.

**Note**: If the data set does not appear in the dropdown menu, double-check your JSON file
for syntax errors, such as missing or extra commas. For additional clues on what might be
wrong, run `log eeg_simulator`.

**Note**: The sampling frequency is inferred from the first two time points in the CSV file.
The difference between consecutive time points is expected to be constant; if there are
gaps in the data, the pipeline will err with a "Samples dropped" message.

#### Monitoring the state of the pipeline

While the pipeline is running, it is good to monitor its state. The state of the pipeline
is shown in the mTMS panel; the statistics bar in the top-right corner of the panel shows,
e.g., the average number of samples processed per second. Both the number of raw and preprocessed
samples should closely match the sampling frequency of the incoming data. If the number
is significantly lower, the pipeline is lagging behind the EEG/EMG data stream and will
eventually start dropping samples.

"Processing time" fields show the time it takes to process a sample in the preprocessor.
If the processing time is too high, the pipeline will not be able to keep up with the incoming
data stream. In theory, the maximum mean processing time should be the inverse of the sampling
frequency of the incoming data, but in practice, already values somewhat lower than that
start congesting the pipeline, indicated by a drop in the number of processed samples per second.

"Decision time" and "End-to-end latency" fields show the previous time when a stimulation
decision was made by the decider and the estimated end-to-end latency of the pipeline for
the sample based on which the decision was made. There should not be large variation in
the end-to-end latency; if there is, the pipeline is not running smoothly.

### Troubleshooting

As a first step in troubleshooting, check the log messages of the pipeline components.
For instance, if the mTMS device does not perform pulses, first check if the decider
is making decisions by running:

```
log decider
```

If it looks like there is a bug in the software, take a screenshot of the log messages
and send it to the authors.

If the system ends up in a state where something does not work as expected, you can
try restarting the pipeline by pressing "Stop session" and then "Start session" on the
mTMS panel. If that does not help, you can try restarting the mTMS device by pressing
"Stop device" and then "Start device" on the mTMS panel. If even that does not help,
you can restart the software by running `restart` on the command line. Finally,
you can restart the computer.
