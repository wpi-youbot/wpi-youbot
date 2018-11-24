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

        ddx = (x - callback.x) * 0.1
        ddy = (y - callback.y) * 0.1

        motion_command = Twist()
        # note that axes are swapped to meet the youbot requirements
        motion_command.linear.x = ddy
        motion_command.linear.y = ddx
        pub.publish(motion_command)

        callback.x = msg.markers[1].translation.x
        callback.y = msg.markers[1].translation.y

        if ddx > callback.ddx_extr:
            callback.ddx_extr = ddx
            print("ddx Extreme is: %f" % (ddx))
            print("%d. %s appears %d times." % (i, key, wordBank[key]))


        if ddy > callback.ddy_extr:
            callback.ddy_extr = ddy



    else:
        callback.x = msg.markers[1].translation.x
        callback.y = msg.markers[1].translation.y


callback.counter = 0
callback.x = 0
callback.ddx_extr = 0
callback.ddy_extr = 0
callback.y = 0
callback.timestamp = 0

# print msg.data
# print("Hello!")


rospy.init_node('vicon_subscriber')
sub = rospy.Subscriber('vicon/markers', Markers, callback)
rospy.spin()
