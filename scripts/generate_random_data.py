import numpy as np
import argparse
import os
import json

def generate_random_data(num_eeg_channels, num_emg_channels, sampling_frequency, duration):
    num_of_samples = int(duration * sampling_frequency)
    timestamps = np.linspace(0, duration, num_of_samples, endpoint=False)

    data = 2 * np.random.rand(num_of_samples, num_eeg_channels + num_emg_channels) - 1
    data_with_timestamps = np.column_stack((timestamps, data))

    return data_with_timestamps

def save_to_csv(output_directory, filename, data, fmt='%.5f'):
    output_path = os.path.join(output_directory, filename)
    np.savetxt(output_path, data, delimiter=",", fmt=fmt)

def save_to_json(output_directory, base_filename, name, num_eeg_channels, num_emg_channels, data_filename):
    json_filename = base_filename + ".json"
    json_data = {
        "name": name,
        "channels": {
            "eeg": num_eeg_channels,
            "emg": num_emg_channels
        },
        "data_file": data_filename,
    }

    output_path = os.path.join(output_directory, json_filename)
    with open(output_path, 'w') as json_file:
        json.dump(json_data, json_file, indent=2)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate random EEG and EMG data and save it to a CSV file and JSON metadata file.")

    parser.add_argument("--eeg_channels", type=int, default=63, help="Number of EEG channels")
    parser.add_argument("--emg_channels", type=int, default=10, help="Number of EMG channels")
    parser.add_argument("--sampling_frequency", type=int, default=5000, help="Sampling frequency in Hz")
    parser.add_argument("--duration", type=int, default=10, help="Duration in seconds")
    parser.add_argument("--output_directory", type=str, default=".", help="Output directory for files")
    parser.add_argument("--output_filename", type=str, default="random_data", help="Output base filename without extension")
    parser.add_argument("--dataset_name", type=str, default="Random data", help="Name of the dataset")

    args = parser.parse_args()

    data = generate_random_data(args.eeg_channels, args.emg_channels, args.sampling_frequency, args.duration)

    data_filename = args.output_filename + ".csv"
    save_to_csv(
        output_directory=args.output_directory,
        filename=data_filename,
        data=data,
    )

    save_to_json(
        output_directory=args.output_directory,
        base_filename=args.output_filename,
        name=args.dataset_name,
        num_eeg_channels=args.eeg_channels,
        num_emg_channels=args.emg_channels,
        data_filename=data_filename,
    )

    print("Random data with {} EEG channels and {} EMG channels saved to {}/{}.csv".format(
        args.eeg_channels,
        args.emg_channels,
        args.output_directory,
        args.output_filename,
    ))
    print("JSON metadata for '{}' saved to {}/{}.json".format(args.dataset_name, args.output_directory, args.output_filename))
