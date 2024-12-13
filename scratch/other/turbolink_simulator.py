import argparse
import random
import socket
import struct
import time

SERVER_IP = "localhost"
SERVER_PORT = 25000


def create_sample_packet(sample_index: int, channel_count) -> bytes:
    token = 0x0050
    sample_counter = sample_index
    trigger_bits = 0x0000
    aux_channels = 8*[0.0]
    eeg_channels = [random.uniform(-400, 400) for _ in range(channel_count)]


    packet_format = f"!LLL8f{channel_count}f"

    packet = struct.pack(packet_format, token, sample_counter, trigger_bits, *aux_channels, *eeg_channels)

    return packet


def main(args):
    print(f"Sampling frequency set to: {args.sampling_frequency}")
    print(f"EEG channel count: {args.channels}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Sending data to socket:")
    try:
        i = 0
        while True:
            data = create_sample_packet(i, args.channels)
            sock.sendto(data, (SERVER_IP, SERVER_PORT))

            time.sleep(1 / args.sampling_frequency)

            if i % int(args.sampling_frequency/2) == 0:
                counter = data[4:8]

                print(struct.unpack("!L", counter)[0])
            i += 1

    except KeyboardInterrupt:
        print("Exiting...")
        sock.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Turbolink real-time EEG device simulator.')

    parser.add_argument("-f", "--sampling-frequency", type=int,
                        help="Sampling frequency (Hz)", default=5000)

    parser.add_argument("-c", "--channels", type=int, help="Number of EEG channels", default=64)

    args = parser.parse_args()
    main(args)
