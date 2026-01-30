# Getting started

## Creating a new project

A new project is created from the user interface by pressing the "+" button next to the project selection dropdown menu in the top-left corner of the panel.

Here are some guidelines for choosing a project name:

- The project name starts with the researcher's name, e.g., "jarmo"
- The words in the name are separated by hyphens, e.g., "jarmo-testing-pulses"

## Running the real-time pipeline

On the control panel, perform the following steps:

- Select the project from the project selection dropdown menu
- Enter the subject ID (e.g., "001")
- Enter the notes (e.g., "Testing pulses")
- Select the protocol from the dropdown menu
- Select the preprocessor (e.g., "Example")
- The preprocessor is bypassed by default; it can be kept that way for now
- Select the decider (e.g., "Example")
- Enable the decider
- For now, you can leave the presenter disabled

After that, there are two options for streaming the EEG/EMG data to the pipeline:

1) Turn on streaming in the EEG/EMG device. The data source selector in the bottom-right
corner should indicate that the device is connected and streaming. After that, press
"Start" on the bottom of the control panel.

2) Using simulated data: in the data source selector, select "Simulator", select a data set from the dropdown menu,
and press "Start" on the bottom of the control panel.

The decider will show in the bottom right corner of the control panel the stimulation
decisions it makes based on the EEG/EMG data.

Press "Stop" on the bottom of the control panel to stop the session.

## Creating custom preprocessor and decider scripts

Once you have the real-time pipeline running with the example scripts, you can start
modifying the scripts to suit your needs.

The preprocessor scripts are located in `~/projects/<project_name>/preprocessor` and the decider
scripts are located in `~/projects/<project_name>/decider`. You can create a copy of `example.py`
in the respective directory with another name and use it as a starting point for your own script.

## Using custom data

The data sets used by EEG simulator are stored in `~/projects/<project_name>/eeg_simulator`.
Each data set is defined by a CSV file containing the data and a JSON file containing the metadata.

For instance, the data set `test_data` would be defined by the file `test_data.csv`, with the JSON file looking like this:

```json
{
    "name": "Test data",
    "data_file": "test_data.csv",
    "session": {
        "sampling_frequency": 1000,
        "num_eeg_channels": 3,
        "num_emg_channels": 1
    }
}
```

Optional fields:
- `loop`: Set to `true` to loop the dataset continuously (default: `false`)
- `pulse_times`: Array of timestamps (in seconds) when pulses were delivered during the session, e.g., `[10.5, 25.3, 41.7]`. Not supported when `loop` is `true`.

The data CSV file should contain the channel data in the following format:

```csv
0.1,0.2,0.3,1.0
0.11,0.21,0.31,1.1
0.12,0.22,0.32,1.2
...
```

Each row is one sample. The columns are EEG channels followed by EMG channels. The number of columns should match `num_eeg_channels + num_emg_channels` in the JSON file.

**Note**: If the data set does not appear in the data source selector, double-check your JSON file
for syntax errors, such as missing or extra commas. For additional clues on what might be
wrong, check the log messages of the EEG simulator.

## Monitoring the state of the pipeline

While the session is running, it is useful to monitor its state. The state of the session
is shown in the control panel; the statistics bar in the top-right corner of the control panel shows,
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

# Troubleshooting

If the system ends up in a state where something does not work as expected, you can
try restarting the session by pressing "Stop" and then "Start" on the bottom of the control panel.
If that does not help, you can restart the software by running `restart-neurosimo` on the command line.
Finally, you can restart the computer.
