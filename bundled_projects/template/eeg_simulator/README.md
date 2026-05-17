# EEG Simulator Datasets

This directory contains datasets for the EEG simulator. Each dataset consists of a JSON metadata file and a CSV data file.

## Dataset Format

### JSON Metadata File

Each dataset is defined by a JSON file with the following structure:

```json
{
  "name": "My dataset",
  "data_file": "my_data.csv",
  "session": {
    "sampling_frequency": 5000,
    "num_eeg_channels": 63,
    "num_emg_channels": 10
  },
  "loop": true,
  "event_file": "my_events.csv"
}
```

### Fields

| Field | Required | Type | Description |
|---|---|---|---|
| `name` | Yes | string | Display name for the dataset |
| `data_file` | Yes | string | Filename of the CSV data file (in the same directory) |
| `session.sampling_frequency` | Yes | integer | Sampling frequency in Hz |
| `session.num_eeg_channels` | Yes | integer | Number of EEG channels |
| `session.num_emg_channels` | Yes | integer | Number of EMG channels |
| `loop` | No | boolean | Whether to loop the dataset when it reaches the end. Defaults to `false`. When `false`, the session is aborted when the dataset ends. |
| `event_file` | No | string | Filename of a CSV file containing event times (in seconds). Not supported together with `loop: true`. |

### CSV Data File

The data file is a CSV where each row is one sample and each column is a channel. Values are comma-separated floating-point numbers with no header row.

The first `num_eeg_channels` columns are EEG channels and the remaining columns are EMG channels. The total number of columns must be at least `num_eeg_channels + num_emg_channels`.

### Event File (Optional)

The event file is a CSV with one event time per line, in seconds. Empty and whitespace-only lines are skipped. When present, the simulator sets `event_trigger` on the sample closest to each event time.

Event files are not supported when `loop` is `true` - if both are specified, the event times are ignored.
