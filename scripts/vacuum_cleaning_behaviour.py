#!/usr/bin/env python  
import roslib
import rospy
import math
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist


class VaccumCleaner:
    def __init__(self):
        rospy.init_node('vaccum_nodes')
        self.pose = Pose()
        # Create a publisher object for sending velocity commands to turtle 1
        self.turtle1_cmd_vel_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Create a publisher object for sending velocity commands to turtle 2
        self.turtle2_cmd_vel_publisher = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

        # This code creates three subscribers in ROS (Robot Operating System) using the Python client library (rospy). 
        # The subscribers are listening to the topics "/turtle1/pose", "/turtle2/pose" for messages of type Pose.
        # The messages received by the subscribers are passed to the callback functions "self.first_turtle_callback", "self.second_turtle_callback",
        # These callbacks are responsible for processing the incoming messages and updating the state of the system as needed.
        self.turtle1_pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.first_turtle_callback)
        self.turtle2_pose_subscriber = rospy.Subscriber('/turtle2/pose', Pose, self.second_turtle_callback)

        # Wait for the service "/turtle1/set_pen" to become available
        rospy.wait_for_service('/turtle1/set_pen')

        # Wait for the service "/turtle2/set_pen" to become available
        rospy.wait_for_service('/turtle2/set_pen')

        try:
            # Create a service proxy for turtle 1's set_pen service
            pen_service_turtle1 = rospy.ServiceProxy('/turtle1/set_pen', SetPen)

            # Create a service proxy for turtle 2's set_pen service
            pen_service_turtle2 = rospy.ServiceProxy('/turtle2/set_pen', SetPen)

            # Call turtle 1's set_pen service to set the pen color to black
            pen_service_turtle1(0, 0, 0, 30, 0)

            # Call turtle 2's set_pen service to set the pen color to black
            pen_service_turtle2(0, 0, 0, 30, 0)

        except rospy.ServiceException as e:
            # Print an error message if the service call failed
            print("%s" % e)

    def first_turtle_callback(self, pose):
        self.first_turtle_point_X = round(pose.x, 1)
        self.first_turtle_point_Y = round(pose.y, 1)

    def second_turtle_callback(self, pose):
        self.second_turtle_point_X = round(pose.x, 1)
        self.second_turtle_point_Y = round(pose.y, 1)

    def control_first_turtle(self):
        twist = Twist()
        linear = 2
        angular = 2
        turtleDistnace = abs(math.sqrt((self.second_turtle_point_X - self.first_turtle_point_X) ** 2 + (
                    self.second_turtle_point_Y - self.first_turtle_point_Y) ** 2))
        if turtleDistnace < 2.0:
            twist.angular.z = -angular
            twist.linear.x = linear

        elif self.first_turtle_point_X > 9.0 or self.first_turtle_point_Y > 9.0 or self.first_turtle_point_X < 2.0 or self.first_turtle_point_Y < 2.0:
            twist.angular.z = -angular
            twist.linear.x = linear
        else:
            twist.angular.z = 0
            twist.linear.x = linear

        self.turtle1_cmd_vel_publisher.publish(twist)

    def control_second_turtle(self):
        twist = Twist()
        linear = 2
        angular = 2
        turtleDistance = abs(math.sqrt((self.second_turtle_point_X - self.first_turtle_point_X) ** 2 + (
                    self.second_turtle_point_Y - self.first_turtle_point_Y) ** 2))

        if turtleDistance < 2.0:
            twist.angular.z = angular
            twist.linear.x = linear
        elif self.second_turtle_point_X > 9.0 or self.second_turtle_point_Y > 9.0 or self.second_turtle_point_X < 2.0 or self.second_turtle_point_Y < 2.0:
            twist.angular.z = angular
            twist.linear.x = linear

        else:
            twist.angular.z = 0
            twist.linear.x = linear

        self.turtle2_cmd_vel_publisher.publish(twist)


# The above code initializes the VaccumCleaner object and creates a loop with a rate of 10 Hz.
# The loop calls the move_turtle1 and move_turtle2 methods to move the turtle robots and then sleeps for the specified rate.
# The loop will continue until the rospy shutdown event is triggered.
if __name__ == '__main__':

    vaccum = VaccumCleaner()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        vaccum.control_first_turtle()
        vaccum.control_second_turtle()
        rate.sleep()
