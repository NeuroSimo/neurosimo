## EEG simulator

### Generating data

The file `random_data.csv` in `example/eeg/raw` directory in repository root contains 0.1 seconds of random values
between 0 and 1. There are 73 values per line (corresponding to 63 EEG channels and 10 EMG channels) with the sampling
frequency of 5 kHz. In addition, the first column is the timestamp.

The data has been generated with the following Python invocation; modify as needed.

```
import numpy as np

sampling_frequency = 5000
duration = 10
num_of_eeg_channels = 63
num_of_emg_channels = 10

num_of_samples = int(duration * sampling_frequency)

timestamps = np.linspace(0, duration, num_of_samples, endpoint=False)

data = 2 * np.random.rand(num_of_samples, num_of_eeg_channels + num_of_emg_channels) - 1
data_with_timestamps = np.column_stack((timestamps, data))

np.savetxt("random_data.csv", data_with_timestamps, delimiter=",", fmt='%.4f')
```
