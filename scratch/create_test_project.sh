#!/bin/bash

# Script to copy test project and generate data with a pulse

set -e

# Check if required environment variables are set
if [ -z "$NEUROSIMO_ROOT" ]; then
    echo "Error: NEUROSIMO_ROOT environment variable is not set"
    exit 1
fi

if [ -z "$PROJECTS_ROOT" ]; then
    echo "Error: PROJECTS_ROOT environment variable is not set"
    exit 1
fi

echo "Copying bundled template project..."
cp -r "$NEUROSIMO_ROOT/bundled_projects/template" "$PROJECTS_ROOT/test"

echo "Adding test project contents..."
cp -r "$NEUROSIMO_ROOT/bundled_projects/test/." "$PROJECTS_ROOT/test/"

echo "Generating project datasets (100Hz, 1kHz, 3kHz, 5kHz)..."

# Generate each dataset individually like project_manager.py does

echo "Generating random_data_100_hz with 100 Hz sampling frequency..."
python3 "$NEUROSIMO_ROOT/scratch/generate_random_data.py" \
    --duration 30 \
    --output_directory "$PROJECTS_ROOT/test/eeg_simulator" \
    --output_filename "random_data_100_hz" \
    --dataset_name "Random, 100 Hz" \
    --sampling_frequency 100 \
    --loop

echo "Generating random_data_1_khz with 1000 Hz sampling frequency..."
python3 "$NEUROSIMO_ROOT/scratch/generate_random_data.py" \
    --duration 30 \
    --output_directory "$PROJECTS_ROOT/test/eeg_simulator" \
    --output_filename "random_data_1_khz" \
    --dataset_name "Random, 1 kHz" \
    --sampling_frequency 1000 \
    --loop

echo "Generating random_data_3_khz with 3000 Hz sampling frequency..."
python3 "$NEUROSIMO_ROOT/scratch/generate_random_data.py" \
    --duration 30 \
    --output_directory "$PROJECTS_ROOT/test/eeg_simulator" \
    --output_filename "random_data_3_khz" \
    --dataset_name "Random, 3 kHz" \
    --sampling_frequency 3000 \
    --loop

echo "Generating random_data_5_khz with 5000 Hz sampling frequency..."
python3 "$NEUROSIMO_ROOT/scratch/generate_random_data.py" \
    --duration 30 \
    --output_directory "$PROJECTS_ROOT/test/eeg_simulator" \
    --output_filename "random_data_5_khz" \
    --dataset_name "Random, 5 kHz" \
    --sampling_frequency 5000 \
    --loop

echo "Generating test data with pulse at 5 seconds..."
python3 "$NEUROSIMO_ROOT/scratch/generate_random_data.py" \
    --duration 30 \
    --output_directory "$PROJECTS_ROOT/test/eeg_simulator" \
    --output_filename "test_data_with_pulse" \
    --dataset_name "Test data with pulse at 5s" \
    --sampling_frequency 5000 \
    --pulse_times 5.0

echo "Test project creation complete!"