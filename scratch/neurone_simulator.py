#!/usr/bin/env python3
"""
NeurOne EEG Simulator

This script simulates UDP packets that the eeg_bridge would recognize as coming from
a Bittium NeurOne EEG device. It sends measurement start, sample data, and measurement
end packets at the specified sampling frequency.
"""

import socket
import struct
import time
import threading
import signal
import sys
import math
import random

# NeurOne protocol constants (from neurone_adapter.h)
class FrameType:
    MEASUREMENT_START = 1
    SAMPLES = 2
    TRIGGER = 3
    MEASUREMENT_END = 4
    HARDWARE_STATE = 5
    JOIN = 128

# Packet field indices
class StartPacketFieldIndex:
    FRAME_TYPE = 0
    MAIN_UNIT_NUM = 1
    RESERVED = 2
    SAMPLING_RATE_HZ = 4
    SAMPLE_FORMAT = 8
    TRIGGER_DEFS = 12
    NUM_CHANNELS = 16
    SOURCE_CHANNEL = 18

class SamplesPacketFieldIndex:
    SAMPLE_PACKET_SEQ_NO = 4
    SAMPLE_NUM_CHANNELS = 8
    SAMPLE_NUM_SAMPLE_BUNDLES = 10
    SAMPLE_FIRST_SAMPLE_INDEX = 12
    SAMPLE_FIRST_SAMPLE_TIME = 20
    SAMPLE_SAMPLES = 28

# Configuration
DEFAULT_PORT = 50000
DEFAULT_SAMPLING_RATE = 500  # Hz
DEFAULT_EEG_CHANNELS = 8
DEFAULT_EMG_CHANNELS = 2
DEFAULT_TRIGGER_PORT = 60000  # Port for receiving triggers from MockLabJackManager
DC_MODE_SCALE = 100  # Scaling factor

class NeurOneSimulator:
    def __init__(self, port=DEFAULT_PORT, sampling_rate=DEFAULT_SAMPLING_RATE,
                 eeg_channels=DEFAULT_EEG_CHANNELS, emg_channels=DEFAULT_EMG_CHANNELS,
                 trigger_port=DEFAULT_TRIGGER_PORT):
        self.port = port
        self.sampling_rate = sampling_rate
        self.eeg_channels = eeg_channels
        self.emg_channels = emg_channels
        self.total_channels = eeg_channels + emg_channels + 1  # +1 for trigger channel
        self.trigger_port = trigger_port

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Bind to a local port for sending
        self.sock.bind(('', 0))

        # Create TCP socket for receiving triggers
        self.trigger_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.trigger_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.trigger_sock.bind(('localhost', self.trigger_port))
        self.trigger_sock.listen(1)

        # EEG device IP (what the bridge expects)
        self.device_ip = 'localhost'

        self.running = False
        self.sample_index = 0

        # Trigger flags for the next sample
        self.next_pulse_trigger = False
        self.next_latency_trigger = False
        self.trigger_lock = threading.Lock()

        # Generate some realistic channel configurations
        self.source_channels = []
        for i in range(self.eeg_channels):
            self.source_channels.append(i + 1)  # EEG channels 1-8

        for i in range(self.emg_channels):
            self.source_channels.append(33 + i)  # EMG channels 33-34

        # Add trigger channel
        self.source_channels.append(65535)  # SOURCE_CHANNEL_FOR_TRIGGER

    def handle_trigger_connection(self):
        """Handle incoming trigger connections in a separate thread"""
        print(f"Listening for triggers on port {self.trigger_port}")
        while self.running:
            try:
                # Accept connection with timeout
                self.trigger_sock.settimeout(1.0)
                client_sock, addr = self.trigger_sock.accept()
                print(f"Accepted trigger connection from {addr}")

                # Handle the connection
                self.handle_client_connection(client_sock)

            except socket.timeout:
                continue
            except OSError:
                # Socket was closed
                break
            except Exception as e:
                print(f"Error accepting trigger connection: {e}")

    def handle_client_connection(self, client_sock):
        """Handle messages from a connected trigger client"""
        try:
            while self.running:
                # Receive trigger message with timeout
                client_sock.settimeout(1.0)
                data = client_sock.recv(1024)

                if not data:
                    break  # Connection closed

                message = data.decode('utf-8').strip()
                print(f"Received trigger: {message}")

                with self.trigger_lock:
                    if message == "pulse_trigger":
                        self.next_pulse_trigger = True
                    elif message == "latency_trigger":
                        self.next_latency_trigger = True

        except socket.timeout:
            pass  # Timeout is expected
        except Exception as e:
            print(f"Error handling trigger connection: {e}")
        finally:
            client_sock.close()

    def int32_to_int24_bytes(self, value):
        """Convert int32 to 3-byte big-endian representation"""
        # Ensure value fits in 24 bits
        value = max(-2**23, min(2**23 - 1, value))
        # Pack as 32-bit, take first 3 bytes (big-endian)
        return struct.pack('>I', value & 0xFFFFFFFF)[1:4]

    def send_measurement_start_packet(self):
        """Send the measurement start packet"""
        packet_size = StartPacketFieldIndex.SOURCE_CHANNEL + 2 * self.total_channels
        packet = bytearray(packet_size)

        # Frame type
        packet[StartPacketFieldIndex.FRAME_TYPE] = FrameType.MEASUREMENT_START

        # Main unit number
        packet[StartPacketFieldIndex.MAIN_UNIT_NUM] = 1

        # Reserved
        packet[StartPacketFieldIndex.RESERVED] = 0

        # Sampling rate (32-bit big-endian)
        struct.pack_into('>I', packet, StartPacketFieldIndex.SAMPLING_RATE_HZ, self.sampling_rate)

        # Sample format (32-bit, details not critical for simulation)
        struct.pack_into('>I', packet, StartPacketFieldIndex.SAMPLE_FORMAT, 1)

        # Trigger definitions (32-bit, simplified)
        struct.pack_into('>I', packet, StartPacketFieldIndex.TRIGGER_DEFS, 0)

        # Number of channels (16-bit big-endian)
        struct.pack_into('>H', packet, StartPacketFieldIndex.NUM_CHANNELS, self.total_channels)

        # Source channels (16-bit big-endian each)
        for i, source_channel in enumerate(self.source_channels):
            offset = StartPacketFieldIndex.SOURCE_CHANNEL + 2 * i
            struct.pack_into('>H', packet, offset, source_channel)

        print(f"Sending measurement start packet: {self.total_channels} channels, {self.sampling_rate} Hz")
        self.sock.sendto(packet, (self.device_ip, self.port))

    def generate_sample_data(self, channel_index, pulse_trigger=False, latency_trigger=False):
        """Generate realistic EEG/EMG sample data"""
        t = self.sample_index / self.sampling_rate

        if channel_index < self.eeg_channels:
            # EEG channels: mix of alpha waves (10 Hz) and some noise
            alpha_freq = 10.0
            base_signal = 50 * math.sin(2 * math.pi * alpha_freq * t)
            noise = random.gauss(0, 5)
            return int((base_signal + noise) * 1000)  # Convert to nV
        elif channel_index < self.eeg_channels + self.emg_channels:
            # EMG channels: higher frequency content
            emg_freq = 50.0
            base_signal = 20 * math.sin(2 * math.pi * emg_freq * t)
            noise = random.gauss(0, 2)
            return int((base_signal + noise) * 1000)  # Convert to nV
        else:
            # Trigger channel: set bits based on received triggers
            trigger_value = 0
            if pulse_trigger:
                trigger_value |= (1 << 3)  # Set bit 3 for pulse trigger
            if latency_trigger:
                trigger_value |= (1 << 1)  # Set bit 1 for latency trigger

            return trigger_value

    def send_sample_packet(self):
        """Send a sample packet"""
        packet_size = SamplesPacketFieldIndex.SAMPLE_SAMPLES + 3 * self.total_channels
        packet = bytearray(packet_size)

        # Frame type
        packet[0] = FrameType.SAMPLES

        # Packet sequence number (32-bit big-endian, simplified)
        struct.pack_into('>I', packet, SamplesPacketFieldIndex.SAMPLE_PACKET_SEQ_NO, self.sample_index % 10000)

        # Number of channels (16-bit big-endian)
        struct.pack_into('>H', packet, SamplesPacketFieldIndex.SAMPLE_NUM_CHANNELS, self.total_channels)

        # Number of sample bundles (16-bit big-endian, must be 1)
        struct.pack_into('>H', packet, SamplesPacketFieldIndex.SAMPLE_NUM_SAMPLE_BUNDLES, 1)

        # First sample index (64-bit big-endian)
        struct.pack_into('>Q', packet, SamplesPacketFieldIndex.SAMPLE_FIRST_SAMPLE_INDEX, self.sample_index)

        # First sample time (64-bit big-endian microseconds)
        current_time_us = int(time.time() * 1_000_000)
        struct.pack_into('>Q', packet, SamplesPacketFieldIndex.SAMPLE_FIRST_SAMPLE_TIME, current_time_us)

        # Check for trigger flags and include them in the sample data
        with self.trigger_lock:
            pulse_trigger = self.next_pulse_trigger
            latency_trigger = self.next_latency_trigger
            # Reset flags after reading
            self.next_pulse_trigger = False
            self.next_latency_trigger = False

        # Sample data (3 bytes per channel, big-endian)
        for i in range(self.total_channels):
            sample_value = self.generate_sample_data(i, pulse_trigger, latency_trigger)
            sample_bytes = self.int32_to_int24_bytes(sample_value)
            offset = SamplesPacketFieldIndex.SAMPLE_SAMPLES + 3 * i
            packet[offset:offset+3] = sample_bytes

        self.sock.sendto(packet, (self.device_ip, self.port))
        self.sample_index += 1

    def send_measurement_end_packet(self):
        """Send the measurement end packet"""
        packet = bytearray(4)
        packet[0] = FrameType.MEASUREMENT_END
        # Rest of the packet is padding/ignored

        print("Sending measurement end packet")
        self.sock.sendto(packet, (self.device_ip, self.port))

    def run(self, duration_seconds=None):
        """Run the simulator for a specified duration or until interrupted"""
        self.running = True
        self.sample_index = 0

        print(f"Starting NeurOne simulator on port {self.port}")
        print(f"Sampling rate: {self.sampling_rate} Hz")
        print(f"EEG channels: {self.eeg_channels}, EMG channels: {self.emg_channels}")
        print(f"Trigger server listening on port {self.trigger_port}")

        # Start trigger handling thread
        trigger_thread = threading.Thread(target=self.handle_trigger_connection)
        trigger_thread.daemon = True
        trigger_thread.start()

        # Send measurement start packet
        self.send_measurement_start_packet()
        time.sleep(0.1)  # Brief pause

        # Calculate sleep time between samples
        sleep_time = 1.0 / self.sampling_rate

        start_time = time.time()
        last_sample_time = start_time

        try:
            while self.running:
                current_time = time.time()

                # Send sample packet
                self.send_sample_packet()

                # Check if we should stop after duration
                if duration_seconds and (current_time - start_time) >= duration_seconds:
                    break

                # Sleep to maintain sampling rate (accounting for processing time)
                processing_time = current_time - last_sample_time
                sleep_duration = max(0, sleep_time - processing_time)
                time.sleep(sleep_duration)

                last_sample_time = time.time()

        except KeyboardInterrupt:
            print("\nInterrupted by user")

        # Send measurement end packet
        self.send_measurement_end_packet()

        print(f"Simulation completed. Sent {self.sample_index} samples")

    def stop(self):
        """Stop the simulator"""
        self.running = False
        try:
            self.trigger_sock.close()
        except:
            pass


def main():
    import argparse

    parser = argparse.ArgumentParser(description='NeurOne EEG Simulator')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT,
                       help=f'UDP port to send packets to (default: {DEFAULT_PORT})')
    parser.add_argument('--trigger-port', type=int, default=DEFAULT_TRIGGER_PORT,
                       help=f'TCP port to listen for triggers (default: {DEFAULT_TRIGGER_PORT})')
    parser.add_argument('--sampling-rate', type=int, default=DEFAULT_SAMPLING_RATE,
                       help=f'Sampling rate in Hz (default: {DEFAULT_SAMPLING_RATE})')
    parser.add_argument('--eeg-channels', type=int, default=DEFAULT_EEG_CHANNELS,
                       help=f'Number of EEG channels (default: {DEFAULT_EEG_CHANNELS})')
    parser.add_argument('--emg-channels', type=int, default=DEFAULT_EMG_CHANNELS,
                       help=f'Number of EMG channels (default: {DEFAULT_EMG_CHANNELS})')
    parser.add_argument('--duration', type=float,
                       help='Duration in seconds to run (default: run until interrupted)')

    args = parser.parse_args()

    simulator = NeurOneSimulator(
        port=args.port,
        sampling_rate=args.sampling_rate,
        eeg_channels=args.eeg_channels,
        emg_channels=args.emg_channels,
        trigger_port=args.trigger_port
    )

    def signal_handler(signum, frame):
        print("\nStopping simulator...")
        simulator.stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        simulator.run(args.duration)
    except Exception as e:
        print(f"Error: {e}")
        simulator.stop()
    finally:
        simulator.sock.close()


if __name__ == '__main__':
    main()