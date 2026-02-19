import numpy as np
import os
import json


class RandomDataGenerator:
    def __init__(self, eeg_channels=63, emg_channels=10, duration=10):
        self.eeg_channels = eeg_channels
        self.emg_channels = emg_channels
        self.duration = duration

    def generate_random_data(self, sampling_frequency):
        """Generate random data for EEG and EMG channels."""
        num_of_samples = int(self.duration * sampling_frequency)
        # Generate random data for EEG and EMG channels (no timestamp column)
        data = 2 * np.random.rand(num_of_samples, self.eeg_channels + self.emg_channels) - 1
        return data

    def save_to_csv(self, output_directory, filename, data, fmt='%.5f'):
        """Save data to CSV file."""
        output_path = os.path.join(output_directory, filename)
        np.savetxt(output_path, data, delimiter=",", fmt=fmt)

    def save_to_json(self, output_directory, base_filename, name, sampling_frequency, data_filename, loop):
        """Save metadata to JSON file."""
        json_filename = base_filename + ".json"
        json_data = {
            "name": name,
            "data_file": data_filename,
            "session": {
                "sampling_frequency": sampling_frequency,
                "num_eeg_channels": self.eeg_channels,
                "num_emg_channels": self.emg_channels,
            },
            "loop": loop,
        }

        output_path = os.path.join(output_directory, json_filename)
        with open(output_path, 'w') as json_file:
            json.dump(json_data, json_file, indent=2)

    def generate_and_save(self, dataset_name, sampling_frequency, output_filename, output_directory, loop=True):
        """Generate random data and save both CSV and JSON files."""
        # Generate the data
        data = self.generate_random_data(sampling_frequency)

        # Save to CSV
        csv_filename = output_filename + ".csv"
        self.save_to_csv(output_directory, csv_filename, data)

        # Save metadata to JSON
        self.save_to_json(
            output_directory=output_directory,
            base_filename=output_filename,
            name=dataset_name,
            sampling_frequency=sampling_frequency,
            data_filename=csv_filename,
            loop=loop
        )

        return csv_filename
