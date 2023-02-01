#!/usr/bin/env python  
import roslib


import rospy
from turtlesim.srv import Spawn

rospy.init_node('spawn_turtle')

# Wait for the service to become available
rospy.wait_for_service('spawn')

# Call the service to spawn a new turtle
spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
response = spawn_turtle(1.0, 3, 1.7, "")

# Check the response to see if the turtle was spawned successfully
if response.success:
    print("Turtle spawned successfully at (x, y) = ({0}, {1})".format(x, y))
else:
    print("Failed to spawn turtle")
