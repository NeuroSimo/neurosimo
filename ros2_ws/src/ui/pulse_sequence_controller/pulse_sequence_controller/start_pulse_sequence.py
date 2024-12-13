import rclpy
from rclpy.node import Node

from ui_interfaces.srv import StartPulseSequence
from event_interfaces.msg import Pulse, EventInfo
from .pulses import generate_pulse, pulse_duration_in_us
from .testResult import TestResult


class StartPulseSequenceNode(Node):
    def __init__(self):
        super().__init__('start_pulse_sequence')
        self.create_service(StartPulseSequence, '/stimulation/start_session', self.start_pulse_sequence_callback)
        self.start_time = self.get_clock().now().microseconds / 1e6
        self.start_delay = 1e-4

        # self.ibi_start_duration = 50
        # self.ibi_end_duration = 50

    def pulse_sequence_is_possible(self, pulse_sequence):
        result = TestResult()
        result.result = True
        return result
        real_iti = pulse_sequence.iti - pulse_sequence.ibi * pulse_sequence.nof_bursts_in_trains

        pulse_durations = []

        pulses = []
        # TODO: different length pulses?
        for i in range(pulse_sequence.nof_pulses_in_bursts):
            pulse = generate_pulse(pulse_sequence.channel_info[0].channel_index, pulse_sequence.channel_info[0].voltage)
            pulse_duration = pulse_duration_in_us(pulse)
            pulse_durations.append(pulse_duration)
            pulses.append(pulse)

        last_pulse_duration = pulse_durations[-1]

        # IBI >= sum(isis) + last pulse
        if not pulse_sequence.ibi >= sum(pulse_sequence.isis) + last_pulse_duration:
            result.result = False
            result.reason = 'Assert failure: IBI >= ' \
                            'sum(isis) + last_pulse_duration: {} >= {} + {}' \
                .format(pulse_sequence.ibi, sum(pulse_sequence.isis), last_pulse_duration)
            return result

        # ITI >= IBI * nofBurstsInTrains
        if not pulse_sequence.iti >= pulse_sequence.ibi * pulse_sequence.nof_bursts_in_trains:
            result.result = False
            result.reason = 'Assert failure: ITI >= ' \
                            'IBI * nof_bursts_in_trains: {} >= {} * {}' \
                .format(pulse_sequence.iti, pulse_sequence.ibi, pulse_sequence.nof_bursts_in_trains)
            return result

        # ISI >= for isi in isis: this pulse duration < isi
        for i in range(len(pulse_durations[:len(pulse_sequence.isis)])):
            if not pulse_sequence.isis[i] > pulse_durations[i]:
                result.result = False
                result.reason = 'Assert failure: isis should be longer than pulse durations (pulse n. {}): {} >= {}' \
                    .format(i + 1, pulse_sequence.isis[i], pulse_durations[i])
                return result

        for channel_info in pulse_sequence.channel_info:
            consumption = self.burst_voltage_consumption(pulses)
            # channel voltage - burst voltage consumption*nofBurstsInTrains > 0
            if not channel_info.voltage - consumption * pulse_sequence.nof_bursts_in_trains > 0:
                result.result = False
                result.reason = 'Not enough voltage for channel {}. ' \
                                'channel_info.voltage - burst_voltage_consumption * nof_bursts_in_trains > 0: ' \
                                '{} - {} * {} > 0' \
                    .format(channel_info.channel_index, channel_info.voltage, consumption,
                            pulse_sequence.nof_bursts_in_trains)
                return result

            # charge_time_from_to(voltage - burst voltage consumption*nofBurstsInTrains, voltage) < REAL_ITI
            if not self.charge_time_from_to(channel_info.voltage - consumption * pulse_sequence.nof_bursts_in_trains,
                                            channel_info.voltage) < real_iti:
                result.result = False
                result.reason = 'Not enough time between trains. ' \
                                'charge_time_from_to(channel_voltage - burst_voltage_consumption *' \
                                ' nof_bursts_in_trains, channel_voltage) ' \
                                '< real_iti: charge_time_from_to({} - {} * {}, {}) < {}' \
                    .format(channel_info.voltage, consumption, pulse_sequence.nof_bursts_in_trains, channel_info.voltage,
                            real_iti)

                return result

        return result

    # TODO
    def charge_time_from_to(self, start_voltage, target_voltage):
        return 100

    # TODO
    def burst_voltage_consumption(self, pulses):
        return 100

    # All times are relative to self.start_time, which is the time when the session command was received
    def calculate_time(self, train_interval, burst_interval, stimulus_interval):
        return self.start_time + self.start_delay + train_interval + burst_interval + stimulus_interval

    def send_pulse(self, event):
        self.get_logger().info('SENDING: ' + str(event))

    def send_charge(self, channel_info):
        charge = {
            'channel': channel_info.channel_index,
            'voltage': channel_info.voltage
        }
        self.get_logger().info('Charging channel {} to {}V'.format(channel_info.channel_index, channel_info.voltage))

    def start_pulse_sequence_callback(self, request, response):
        self.get_logger().info('Incoming request')
        self.get_logger().info(str(request))
        pulse_sequence = request.session.pulse_sequence
        sequence_is_possible = self.pulse_sequence_is_possible(pulse_sequence)
        if not sequence_is_possible.result:
            response.success = False
            response.status = sequence_is_possible.reason
            response.sequence = []
            return response

        # TODO: wait for charging to finish. Loop until mTMS device publishes to a topic that all channels are ready or
        #  until async (or sync?) service calls finish?
        for channel_info in pulse_sequence.channel_info:
            self.send_charge(channel_info)

        id = 0
        train_interval = 0
        burst_interval = 0

        sequence = []

        for train_index in range(pulse_sequence.nof_trains):
            for burst_index in range(pulse_sequence.nof_bursts_in_trains):
                for pulse_index in range(pulse_sequence.nof_pulses_in_bursts):

                    # Send event for each enabled channel
                    for channel_info in pulse_sequence.channel_info:
                        channel_index = channel_info.channel_index
                        voltage = channel_info.voltage

                        pulse = generate_pulse(channel_index, voltage)

                        # isi for the first pulse is 0 as there are no preceding pulses to "wait" for
                        isi = 0 if pulse_index == 0 else pulse_sequence.isis[pulse_index - 1]

                        event_info = EventInfo()
                        event_info.id = id
                        # TODO: Bitrotten: 'wait_for_trigger' has been replaced with 'execution_condition'.
                        event_info.wait_for_trigger = False
                        event_info.execution_time = self.calculate_time(train_interval, burst_interval, isi)

                        pulse = Pulse()
                        pulse.waveform = pulse
                        pulse.channel = channel_index
                        pulse.event_info = event_info

                        self.send_pulse(pulse)
                        sequence.append(pulse)

                        id += 1
                burst_interval += pulse_sequence.ibi

            train_interval += pulse_sequence.iti
        self.get_logger().info(str(sequence))
        response.success = True
        response.sequence = sequence

        return response


def main():
    rclpy.init()

    start_sequence_node = StartPulseSequenceNode()

    rclpy.spin(start_sequence_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
