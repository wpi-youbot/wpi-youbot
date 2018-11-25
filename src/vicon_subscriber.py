#! /usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist  # message used by /cmd_vel topic
from geometry_msgs.msg import TransformStamped  # message used by /vicon/kukay/kukay topic
from youbot_functions import *
from trajectory_generation import *

states = []  # robot states over time (x, y, theta, dx, dy, dtheta)

pub = rospy.Publisher('/gazebo/cmd_vel', Twist, queue_size=1)


# TODO: Enable correct robot simulation when rotation between robot coordinate frame and world frame occurs

def callback(msg):
    global states
    callback.counter += 1

    if callback.counter > 1:  # the second and any other call of this callback
        x = msg.transform.translation.x  # get the vicon data from the message
        y = msg.transform.translation.y
        pid(states)

        # get the robot orientation [rad], 1 rotation DOF
        quat_z = msg.transform.rotation.z
        quat_w = msg.transform.rotation.w
        theta = 2.0 * math.atan2(quat_z, quat_w)

        dx = (x - states[-1][0])
        dy = (y - states[-1][1])
        dtheta = (theta - states[-1][2])
        print("x: %f y: %f theta: %f dx: %f dy: %f | dtheta: %f" % (x, y, theta, dx, dy, dtheta))

        motion_command = Twist()

        safe_linear_speed = 40.0  # TODO: this value needs some adjustment
        safe_angular_speed = 3.0  # rad/s # TODO: verify max value with kuka manual
        if (abs(dy) * 100 < safe_linear_speed) and (abs(dx) * 100 < safe_linear_speed) and (
                abs(dtheta) * 100 < safe_angular_speed):
            # pass vel values to the message
            # print("Hello OK")
            motion_command.linear.x = dy * 100  # note that axes are swapped to meet the youbot requirements
            motion_command.linear.y = dx * 100
            motion_command.angular.z = dtheta * 100

            # Alternatively do the PID here

            # update last valid velocities
            states.append([x, y, theta, dx, dy, dtheta])

            # update last valid positions
            pub.publish(motion_command)

        else:
            # use last valid vel values for current message
            dx = states[-1][3]
            dy = states[-1][4]
            dtheta = states[-1][5]

            x = states[-1][0] + dx  # update position adding last valid translation/rotation
            y = states[-1][1] + dy
            theta = states[-1][2] + dtheta
            states.append([x, y, theta, dx, dy, dtheta])
            # print(states[-1][:])

            motion_command.linear.x = dy * 100
            motion_command.linear.y = dx * 100
            motion_command.angular.z = dtheta * 100

            # print("my_old_x: %f my_old_y: %f" % (x, y))
            pub.publish(motion_command)
            print("Disaster: vicon outlier")

    else:
        print("**** INITIAL CASE - NO PREVIOUS STATE ****")
        x = msg.transform.translation.x
        y = msg.transform.translation.y

        quat_z = msg.transform.rotation.z
        quat_w = msg.transform.rotation.w
        theta = 2.0 * math.atan2(quat_z, quat_w)

        # pass robot initial positions and assume zero velocities
        states.append([x, y, theta, 0, 0, 0])


callback.counter = 0

rospy.init_node('vicon_subscriber')
sub = rospy.Subscriber('vicon/kukay/kukay', TransformStamped, callback)

rospy.spin()
