#! /usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist

rospy.init_node('Planner')

pub = rospy.Publisher('/gazebo/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(2)

motion_command = Twist()

# scale = 0.05
scale = 0.1
# motions = [-10:10] + [10:-10]
# motions = range(-10,1,10) + range(10,-1,-10)

motions = list(range(0, 10, 1)) + list(range(10, -10, -1)) + list(range(-10, 0, 1))

while not rospy.is_shutdown():
    for vel in motions:
        # motion_command.linear.x = -0.77
        # motion_command.angular.z = vel * scale
        # pub.publish(motion_command)
        rate.sleep()

        # time.sleep(.500)
