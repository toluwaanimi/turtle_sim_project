#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2

msg = """
Borders of the wall to avoid :
x = [0.75, 10.5]
y = [0.75, 10.5]
Move Turtle using:
    ^
<       >   
arrow keys
  
"""


class Avoid:
    def __init__(self):
        rospy.init_node('Turtlesim_Avoid_collision', anonymous=True)
        # Publisher which will publish to '/turtle1/cmd_vel'
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subs = rospy.Subscriber('/turtle1/pose',Pose, self.pose_update)
        self.rate = rospy.Rate(100)

        
        rospy.loginfo(msg)
    

    #Update the destination point when a messages is published 
    def pose_update(self,data):  
        try:
            
            self.pose = data
            self.pose.x = round(self.pose.x,2)
            self.pose.y = round(self.pose.y,2)

            cmd_value=Twist()

            #Function for avoiding the wall and turning the turtle towards center
            def avoid_wall():
                
                #turning the turtle to centre of the window 
                deltaX = 5.5 - self.pose.x
                deltaY = 5.5 - self.pose.y
                #Returns the values in radians
                Des_Heading = atan2(deltaY, deltaX)
                angle_to_rotate = Des_Heading - self.pose.theta
                #add small linear velocity if turtle touches the wall, it turns and move a bit
                cmd_value.linear.x = 0.2
                cmd_value.angular.z = 7* angle_to_rotate

                #Publishes the Twist values to cmd_vel 
                self.pub.publish(cmd_value)
                self.rate.sleep()
        
            #Checks the turtle is touching the wall
            if (self.pose.x <= x[0] ):
                
                rospy.loginfo("I hit the left wall")
                avoid_wall()


            if (self.pose.y <= y[0]):
                rospy.loginfo("I hit the bottom wall")
                avoid_wall()


            if (self.pose.x >x[1]):
                rospy.loginfo("I hit the Right wall")
                avoid_wall()

            if (self.pose.y >y[1] ) :

                rospy.loginfo("I hit the Top wall")
                avoid_wall()

        except Exception as e2: #Error in Updating Pose
            print(e2)

if __name__ == '__main__':
    try:
        #Borders for the wall 
        x = [0.75, 10.5]
        y = [0.75, 10.5]

        #testing the Avoid collision Class 
        Avoid()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass