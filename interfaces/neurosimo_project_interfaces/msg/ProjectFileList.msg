# The files of a given type available in a project.
#
# The project is named because these lists are published independently of the system
# configuration that determines the active project: a subscriber that has just learned of a
# project change may still be holding the list of the previous project, and a subscriber
# starting up receives the latched list and the latched configuration in either order.
# Naming the project lets subscribers tell which of the two they have.

string project
string[] filenames
