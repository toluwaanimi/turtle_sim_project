#!/usr/bin/env python  
import roslib
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
    
    
class AutonomousNavigation:
   
    def __init__(self):
        #This line initializes a ROS node named "autonomous_navigation" using the rospy library in Python.
        #The anonymous argument is set to False, indicating that the node is not anonymous and its name must be unique in the ROS network.
        rospy.init_node('autonomous_navigation', anonymous=False) 
   
        # Create a publisher object for sending velocity commands to the turtle simulation
        self.turtle_velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
        # Initialize the turtle's pose object with the default values
        self.pose = Pose()
        # Set the rate of the loop to 10 Hz
        self.rate = rospy.Rate(10)
        # Subscribe to the turtle's pose topic to receive updates on the turtle's position and orientation
        self.turtle_pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.resolvePostData) 



    def resolvePostData(self, inputData):
           self.pose = inputData
           self.pose.x = round(self.pose.x, 4)
           self.pose.y = round(self.pose.y, 4)
        
    def compute_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))


    def robotAngleChange(self, destination):
        return atan2(destination.y - self.pose.y, destination.x - self.pose.x)

    def move_to_autonomous(self):
        # Initialize a variable to store the desired goal pose for the turtle
        destinationPose = Pose()
   
        # Prompt the user to input the desired x and y values for the goal pose
        destinationPose.x = float(input("Enter the desired x value for the destination: "))
        destinationPose.y = float(input("Enter the desired y value for the destination: "))

        destination_tolerance = 0.5

        # Initialize a Twist message to store the velocity commands for the turtle
        twiseVelocity = Twist()
        while sqrt(pow((destinationPose.x - self.pose.x), 2) + pow((destinationPose.y - self.pose.y), 2)) >= destination_tolerance:
               twiseVelocity.linear.x = 1.5 * self.compute_distance(destinationPose)
               twiseVelocity.linear.y = 0
               twiseVelocity.linear.z = 0
   
               twiseVelocity.angular.x = 0
               twiseVelocity.angular.y = 0
               twiseVelocity.angular.z = 6 * (self.robotAngleChange(destinationPose) - self.pose.theta)
   
               self.turtle_velocity_publisher.publish(twiseVelocity)
   
               self.rate.sleep()
   
        twiseVelocity.linear.x = 0
        twiseVelocity.angular.z = 0
        self.turtle_velocity_publisher.publish(twiseVelocity)
   
        rospy.spin()

    

# Check if this Python script is being run as the main program
if __name__ == '__main__':
    try:
        # Create an instance of the AutonomousNavigation class
        autonomous_navigation = AutonomousNavigation()
        
        # Call the move_to_autonomous method to start the autonomous navigation
        autonomous_navigation.move_to_autonomous()
    except rospy.ROSInterruptException:
        # Handle the exception that occurs when the ROS node is interrupted
        pass


