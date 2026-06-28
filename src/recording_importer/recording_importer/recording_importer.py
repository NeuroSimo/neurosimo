import os
import json
import numpy as np
import mne
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from pathlib import Path

from neurosimo_project_interfaces.srv import ImportRecording
from neurosimo_system_interfaces.msg import GlobalConfig


class RecordingImporterNode(Node):
    def __init__(self):
        super().__init__('recording_importer')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        self.active_project = None
        self.projects_root = '/app/projects'

        # Subscribe to global config to track active project
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.global_config_subscription = self.create_subscription(
            GlobalConfig,
            '/neurosimo/global_configurator/config',
            self.global_config_callback,
            qos
        )

        # Import service
        self.create_service(
            ImportRecording,
            '/neurosimo/eeg_simulator/import_recording',
            self.import_recording_callback,
            callback_group=self.callback_group
        )

        self.logger.info('Recording importer node initialized')

    def global_config_callback(self, msg):
        self.active_project = msg.active_project

    def import_recording_callback(self, request, response):
        """Import a BrainVision recording and convert to eeg_simulator CSV format."""
        filename = request.filename

        if not self.active_project:
            response.success = False
            response.dataset_filename = ''
            return response

        # Construct the path to the file in external_recordings
        external_recordings_dir = os.path.join(
            self.projects_root, self.active_project, 'external_recordings'
        )
        vhdr_path = os.path.join(external_recordings_dir, filename)

        if not os.path.exists(vhdr_path):
            self.logger.error(f'File not found: {filename}')
            response.success = False
            response.dataset_filename = ''
            return response

        try:
            # Read the BrainVision file using MNE
            self.logger.info(f'Reading BrainVision file: {vhdr_path}')
            raw = mne.io.read_raw_brainvision(vhdr_path, preload=True)

            # Extract channel info
            ch_names = raw.ch_names
            ch_types = raw.get_channel_types()

            eeg_channels = [ch for ch, ch_type in zip(ch_names, ch_types) if ch_type == 'eeg']
            emg_channels = [ch for ch, ch_type in zip(ch_names, ch_types) if ch_type == 'emg']

            num_eeg_channels = len(eeg_channels)
            num_emg_channels = len(emg_channels)

            # If no EMG channels found, treat all non-EEG channels as EMG
            if num_emg_channels == 0:
                non_eeg_channels = [ch for ch, ch_type in zip(ch_names, ch_types) if ch_type != 'eeg']
                num_emg_channels = len(non_eeg_channels)
                # Reorder: EEG channels first, then the rest
                pick_order = eeg_channels + non_eeg_channels
            else:
                pick_order = eeg_channels + emg_channels

            # Pick and reorder channels (EEG first, then EMG/other)
            raw_reordered = raw.pick(pick_order)

            # Get data as numpy array (channels x samples) -> transpose to (samples x channels)
            data = raw_reordered.get_data().T

            # Convert from Volts to microVolts for EEG data readability
            # MNE stores data in SI units (Volts), eeg_simulator expects microVolts
            data[:, :num_eeg_channels] *= 1e6
            if num_emg_channels > 0:
                data[:, num_eeg_channels:] *= 1e6

            sampling_frequency = raw.info['sfreq']
            duration = raw.n_times / sampling_frequency

            # Generate output filename based on input
            base_name = Path(vhdr_filename).stem
            csv_filename = f'{base_name}.csv'
            json_filename = f'{base_name}.json'

            # Output to eeg_simulator directory
            eeg_simulator_dir = os.path.join(
                self.projects_root, self.active_project, 'eeg_simulator'
            )
            os.makedirs(eeg_simulator_dir, exist_ok=True)

            # Save CSV data
            csv_path = os.path.join(eeg_simulator_dir, csv_filename)
            self.logger.info(f'Writing CSV data: {csv_path} ({data.shape[0]} samples, {data.shape[1]} channels)')
            np.savetxt(csv_path, data, delimiter=',', fmt='%.5f')

            # Save JSON metadata
            json_data = {
                'name': base_name,
                'data_file': csv_filename,
                'session': {
                    'sampling_frequency': int(sampling_frequency),
                    'num_eeg_channels': num_eeg_channels,
                    'num_emg_channels': num_emg_channels,
                },
                'loop': False,
            }

            # Add event file if there are annotations
            if len(raw.annotations) > 0:
                events_filename = f'{base_name}_events.csv'
                events_path = os.path.join(eeg_simulator_dir, events_filename)

                # Extract event onset times in seconds
                event_times = raw.annotations.onset
                np.savetxt(events_path, event_times, fmt='%.6f')

                json_data['event_file'] = events_filename
                self.logger.info(f'Written {len(event_times)} events to {events_filename}')

            json_path = os.path.join(eeg_simulator_dir, json_filename)
            with open(json_path, 'w') as f:
                json.dump(json_data, f, indent=2)

            self.logger.info(
                f'Successfully imported {filename}: '
                f'{num_eeg_channels} EEG + {num_emg_channels} EMG channels, '
                f'{sampling_frequency} Hz, {duration:.1f}s'
            )

            response.success = True
            response.dataset_filename = json_filename
            return response

        except Exception as e:
            self.logger.error(f'Failed to import {filename}: {e}')
            response.success = False
            response.dataset_filename = ''
            return response


def main(args=None):
    rclpy.init(args=args)
    node = RecordingImporterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
