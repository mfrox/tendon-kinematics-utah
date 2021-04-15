# Example Applications

This folder has example applications that provide useful examples to learn
from, but will not compile because this repository is a subset of the full
repository and does not contain all of the pieces necessary to compile these
examples.


## plan_with_haptic

This example takes haptic input, listens for a button press on the haptic pen
device, marks that as a goal position, motion plans from the current position
to that goal position, and then waits for the next button press.  It publishes
the shapes to RViz2 over ROS2.
