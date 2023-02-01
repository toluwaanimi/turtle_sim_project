#!/usr/bin/env

import rospy
from geometry_msgs.msg import Twist
import getch

print(" UP/DOWN/LEFT/RIGHT KEY IN KEYBOARD")
print(" A=FORWARD SPEED/B=BACKWARD SPEED/C=RIGHT ANGULAR SPEED/D=LEFT ANGULAR SPEED ")

# initiated "turtle_teleoperator" and publish to cmd_vel

pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
rospy.init_node('turtle_teleoperator', anonymous=False)
rate = rospy.Rate(10)

# linear and angular speed given as 1.0 and 0.5 respectively
linear_speed = 1.0
angular_speed = 0.5

# variable linear and angular speed

variable_linear_speed = linear_speed * 2
variable_angular_speed = angular_speed * 2


# teleoperator () send linear and angular values to move_cmd

def teleoperator():
    try:

        while not rospy.is_shutdown():

            k = ord(getch.getch())

            if k == 65:
                rospy.loginfo("Up")
                move_turtle(linear_speed, 0.0)

            if k == 66:
                rospy.loginfo("Down")
                move_turtle(-linear_speed, 0.0)

            if k == 67:
                rospy.loginfo("Right")
                move_turtle(0.0, -angular_speed)

            if k == 68:
                rospy.loginfo("Left")
                move_turtle(0.0, angular_speed)

            if k == 119:
                rospy.loginfo("forward speed ")
                move_turtle(variable_linear_speed, 0.0)

            if k == 115:
                rospy.loginfo("backward speed")
                move_turtle(-variable_linear_speed, 0.0)

            if k == 100:
                rospy.loginfo("Right Angular speed")
                move_turtle(0.0, -variable_angular_speed)
            if k == 97:
                rospy.loginfo("left Angular speed ")
                move_turtle(0.0, variable_angular_speed)


    except Exception as error:
        print(error)

    # defined move_turtle and add values to move_cmd


def move_turtle(linear, angular):
    move_cmd = Twist()

    move_cmd.linear.x = linear

    move_cmd.angular.z = angular

    pub.publish(move_cmd)

    rate.sleep()


# main function which call initialise()

if __name__ == '__main__':
    try:
        teleoperator()

    except rospy.ROSInterruptException:
        pass
