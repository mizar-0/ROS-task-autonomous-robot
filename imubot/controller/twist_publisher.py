#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


def twist_publisher(control_effort: Float64):
    control = control_effort.data

    cmd = Twist()
    cmd.linear.x = 3.5
    cmd.angular.z = control

    pub.publish(cmd)

    



if __name__ == "__main__":
    rospy.init_node("twist_publisher")
    sub = rospy.Subscriber("control_effort", Float64, callback=twist_publisher)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.loginfo("twist_publisher is live")

    rospy.spin()

