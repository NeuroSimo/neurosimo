import os

from project_interfaces.srv import ListProjects, SetActiveProject
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile


class ProjectManagerNode(Node):
    PROJECTS_ROOT = '/app/projects'

    def __init__(self):
        super().__init__('project_manager_node')

        self.logger = self.get_logger()

        # Create service for listing projects.

        self.service = self.create_service(
            ListProjects,
            '/projects/list',
            self.list_projects_callback,
        )

        # Create service for setting active project.

        self.service = self.create_service(
            SetActiveProject,
            '/projects/active/set',
            self.set_active_project_callback,
        )

        # Create publisher for publishing active project.
        qos_persist_latest = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.active_project_publisher = self.create_publisher(String, "/projects/active", qos_persist_latest)

        # Set active project initially to the first project on the list.
        projects = self.list_projects()
        if projects:
            self.set_active_project(projects[0])

    def set_active_project(self, active_project):
        self.active_project = active_project

        msg = String()
        msg.data = active_project
        self.active_project_publisher.publish(msg)

    def list_projects(self):
        all_projects = [d for d in os.listdir(self.PROJECTS_ROOT) if os.path.isdir(os.path.join(self.PROJECTS_ROOT, d))]

        # If 'example' project exists, list it first to make UI default to it.
        if "example" in all_projects:
            all_projects.remove("example")
            return ["example"] + sorted(all_projects)
        else:
            return sorted(all_projects)

    def list_projects_callback(self, request, response):
        try:
            # List directories in PROJECTS_ROOT
            response.projects = self.list_projects()

            self.logger.info("Projects successfully listed.")
            response.success = True

        except Exception as e:
            self.logger.error("Error listing projects: {}.".format(e))
            response.success = False

        return response

    def set_active_project_callback(self, request, response):
        project = request.project

        # Check if the specified project exists
        if project in [d for d in os.listdir(self.PROJECTS_ROOT) if os.path.isdir(os.path.join(self.PROJECTS_ROOT, d))]:
            self.set_active_project(project)

            self.logger.info("Project successfully set to: {}.".format(project))
            response.success = True
        else:
            self.logger.error("Project does not exist: {}.".format(project))
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)

    project_manager_node = ProjectManagerNode()

    rclpy.spin(project_manager_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
