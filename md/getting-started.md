# Getting started

## Creating a new project

A new project is created from the command line (ctrl+alt+t) by running the following command:

```
create-project <project_name>
```

After that, the project can be opened from the NeuroSimo panel ([http://localhost:3000](http://localhost:3000)).

Here are some guidelines for choosing a project name:

- The project name starts with the researcher's name, e.g., "jarmo"
- The words in the name are separated by hyphens, e.g., "jarmo-testing-pulses"

## Running the real-time pipeline

On the panel, perform the following steps:

- Select the project from the dropdown menu
- Select the preprocessor ("Example" is a good starting point)
- Preprocessor is bypassed by default; it can be kept that way for now
- Select the decider ("Example" is a good starting point here as well)
- Enable the decider
- For now, you can leave "Presenter" disabled

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
log-neurosimo preprocessor
```

will show and follow the log messages of the preprocessor. However, the default
preprocessor only passes the data through, so there is not much to see there.

Instead, you can run

```
log-neurosimo decider
```

to follow the log messages of the decider. The decider will print out the stimulation
decisions it makes based on the EEG/EMG data. Following the log will give an idea
of what the decider does: the example Decider gives a stimulation decision every
two seconds.

Press "Stop session" on the panel to stop the pipeline from running.

## Creating custom preprocessor and decider scripts

Once you have the real-time pipeline running with the example scripts, you can start
modifying the scripts to suit your needs.

The preprocessor scripts are located in `~/projects/<project_name>/preprocessor` and the decider
scripts are located in `~/projects/<project_name>/decider`. For instance, you can create a
copy of `example.py` with another name and use it as a starting point for your own script.

**Note**: The panel automatically updates the scripts in the `preprocessor` and
`decider` dropdown menus when the contents of the directories are modified. That is, after
copying `example.py` to another name, you can immediately start using the new script
by selecting it from the dropdown menu in the panel.

**Note**: When the real-time pipeline is running, the preprocessor and decider modules
automatically reload the script when it is modified. This means that you can modify the
scripts while the pipeline is running, and the changes will take effect immediately after
saving the script. However, beware that the state of the script is reset when it is reloaded.

## Using custom data

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

After creating a new data set, you can select it from the dropdown menu in the panel
where it should automatically appear.

**Note**: If the data set does not appear in the dropdown menu, double-check your JSON file
for syntax errors, such as missing or extra commas. For additional clues on what might be
wrong, run `log eeg_simulator`.

**Note**: The sampling frequency is inferred from the first two time points in the CSV file.
The difference between consecutive time points is expected to be constant; if there are
gaps in the data, the pipeline will err with a "Samples dropped" message.

## Monitoring the state of the pipeline

While the pipeline is running, it is good to monitor its state. The state of the pipeline
is shown in the panel; the statistics bar in the top-right corner of the panel shows,
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

As a first step in troubleshooting, check the log messages of the pipeline components.
For instance, if the TMS device does not deliver pulses, first check if the decider
is making decisions by running:

```
log decider
```

If it looks like there is a bug in the software, take a screenshot of the log messages
and send it to the authors.

If the system ends up in a state where something does not work as expected, you can
try restarting the pipeline by pressing "Stop session" and then "Start session" on the
panel. If that does not help, you can restart the software by running `restart-neurosimo`
on the command line. Finally, you can restart the computer.
