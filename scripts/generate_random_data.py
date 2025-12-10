import numpy as np
import argparse
import os
import json

def generate_random_data(num_eeg_channels, num_emg_channels, sampling_frequency, duration):
    num_of_samples = int(duration * sampling_frequency)

    # Generate random data for EEG and EMG channels (no timestamp column)
    data = 2 * np.random.rand(num_of_samples, num_eeg_channels + num_emg_channels) - 1

    return data

def save_to_csv(output_directory, filename, data, fmt='%.5f'):
    output_path = os.path.join(output_directory, filename)
    np.savetxt(output_path, data, delimiter=",", fmt=fmt)

def save_to_json(output_directory, base_filename, name, sampling_frequency, num_eeg_channels, num_emg_channels, data_filename, loop):
    json_filename = base_filename + ".json"
    json_data = {
        "name": name,
        "data_file": data_filename,
        "session": {
            "sampling_frequency": sampling_frequency,
            "num_eeg_channels": num_eeg_channels,
            "num_emg_channels": num_emg_channels,
        },
        "loop": loop,
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
    parser.add_argument("--loop", action="store_true", help="Whether to loop the dataset when reaching the end")

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
        sampling_frequency=args.sampling_frequency,
        num_eeg_channels=args.eeg_channels,
        num_emg_channels=args.emg_channels,
        data_filename=data_filename,
        loop=args.loop,
    )

    print("Random data with {} EEG channels and {} EMG channels saved to {}/{}.csv".format(
        args.eeg_channels,
        args.emg_channels,
        args.output_directory,
        args.output_filename,
    ))
    print("JSON metadata for '{}' saved to {}/{}.json".format(args.dataset_name, args.output_directory, args.output_filename))
