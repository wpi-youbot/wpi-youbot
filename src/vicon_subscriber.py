#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

# from std_msgs.msg import Int32
from vicon_bridge.msg import Markers

pub = rospy.Publisher('/gazebo/cmd_vel', Twist, queue_size=1)


def callback(msg):
    callback.counter += 1
    # callback.timestamp = msg.

    if callback.counter > 1:
        x = msg.markers[1].translation.x
        y = msg.markers[1].translation.y

        # [mm] * 1000 *
        # ddx = (x - callback.x) * 0.001
        ddx = (x - callback.x) * 0.1
        ddy = (y - callback.y) * 0.1

        motion_command = Twist()
        motion_command.linear.x = ddx
        motion_command.linear.y = ddy
        pub.publish(motion_command)

        callback.x = msg.markers[1].translation.x
        callback.y = msg.markers[1].translation.y

    else:
        callback.x = msg.markers[1].translation.x
        callback.y = msg.markers[1].translation.y


callback.counter = 0
callback.x = 0
callback.y = 0
callback.timestamp = 0

# print msg.data
# print("Hello!")


rospy.init_node('vicon_subscriber')
sub = rospy.Subscriber('vicon/markers', Markers, callback)
rospy.spin()
