import os
import re
from datetime import datetime

from experiment_interfaces.srv import LogTrial
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile


class TrialLoggerNode(Node):

    PROJECTS_ROOT = '/app/projects/'

    TRIAL_COLUMNS = [
        "Trial index",
        "Number of attempts",
        "Trial start time (s)",
        "Number of targets",
        "x (mm)",
        "y (mm)",
        "Angle (deg)",
        "Intensity (V/m)",
        "x (mm, second target)",
        "y (mm, second target)",
        "Angle (deg, second target)",
        "Intensity (V/m, second target)",
        "MEP amplitude (uV)",
        "MEP latency (s)",
    ]

    def __init__(self):
        super().__init__('trial_logger_node')

        self.logger = self.get_logger()

        # Create service for logging trial.

        self.service = self.create_service(
            LogTrial,
            '/trial/log',
            self.log_trial_callback,
        )

        # Create subscriber for active project.
        qos_persist_latest = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.active_project_subscriber = self.create_subscription(String, '/projects/active', self.handle_active_project_subscription, qos_persist_latest)

    def open_log_file(self, metadata):
        subject_name = self.sanitize_filename(metadata.subject_name)
        experiment_name = self.sanitize_filename(metadata.experiment_name)

        basepath = os.path.join(self.PROJECTS_ROOT, self.project, 'csv')

        # Ensure the directory exists
        if not os.path.exists(basepath):
            os.makedirs(basepath)

        # The pattern below matches filenames with the format:
        #
        # YYYY-DD-MM-hh-mm-ss_[experiment_name]_[subject-name].csv
        pattern = re.compile(r"(\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2})_" + re.escape(experiment_name) + "_" + re.escape(subject_name) + r"\.csv")

        matching_files = []
        for filename in os.listdir(basepath):
            if pattern.match(filename):
                matching_files.append(filename)

        # Sort the files based on their timestamp in descending order
        matching_files.sort(reverse=True)

        if matching_files:
            # Take the file with the latest timestamp
            filename = matching_files[0]
            filepath = os.path.join(basepath, filename)
            mode = 'a+'

            new_file_created = False
        else:
            # Create a new file
            current_timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            filename = f"{current_timestamp}_{experiment_name}_{subject_name}.csv"
            filepath = os.path.join(basepath, filename)
            mode = 'w+'

            new_file_created = True
            self.get_logger().info('New log file created!')

        file = open(filepath, mode)
        self.get_logger().info('Logging into the file: {}.'.format(filename))

        return file, new_file_created

    def sanitize_filename(self, filename):
        # Replace any characters not in the whitelist with an underscore. The whitelist consists
        # of alphanumeric characters, period, and hyphen.
        return re.sub(r"[^a-zA-Z0-9\.\-]", "_", filename)

    def write_header(self, file):
        header = ";".join(self.TRIAL_COLUMNS) + '\n'
        file.write(header)

    def log_trial_row(self, file, trial_number, trial, trial_result, num_of_attempts):
        num_of_targets = len(trial.targets)
        assert num_of_targets <= 2, "Does not support more than two targets."

        first_target = trial.targets[0]
        if num_of_targets == 2:
            second_target = trial.targets[1]
        else:
            second_target = None

        row = "{};{};{:.3f};{};{};{};{};{};{};{};{};{};{:.1f};{:.4f}\n".format(
            trial_number,
            num_of_attempts,
            trial_result.actual_start_time,
            num_of_targets,
            first_target.displacement_x,
            first_target.displacement_y,
            first_target.rotation_angle,
            first_target.intensity,
            second_target.displacement_x if second_target else '',
            second_target.displacement_y if second_target else '',
            second_target.rotation_angle if second_target else '',
            second_target.intensity if second_target else '',
            trial_result.mep.amplitude,
            trial_result.mep.latency,
        )
        file.write(row)

    def log_trial(self, metadata, trial_number, trial, trial_result, num_of_attempts):
        file, new_file_created = self.open_log_file(metadata)

        # If the file was just created, write the header as the first line.
        if new_file_created:
            self.write_header(file)

        self.log_trial_row(
            file=file,
            trial_number=trial_number,
            trial=trial,
            trial_result=trial_result,
            num_of_attempts=num_of_attempts,
        )

    def log_trial_callback(self, request, response):
        metadata = request.metadata
        trial_number = request.trial_number
        trial = request.trial
        trial_result = request.trial_result
        num_of_attempts = request.num_of_attempts

        self.get_logger().info('Logging a trial with the index {}.'.format(trial_number))

        self.log_trial(
            metadata=metadata,
            trial_number=trial_number,
            trial=trial,
            trial_result=trial_result,
            num_of_attempts=num_of_attempts,
        )
        response.success = True

        self.get_logger().info('Done.')

        return response

    def handle_active_project_subscription(self, msg):
        self.project = msg.data

        self.get_logger().info('Changed project to: {}.'.format(self.project))

def main(args=None):
    rclpy.init(args=args)

    trial_logger_node = TrialLoggerNode()

    rclpy.spin(trial_logger_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
