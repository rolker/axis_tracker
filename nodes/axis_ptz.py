#!/usr/bin/env python3

import rospy
import axis_tracker.axis

from geometry_msgs.msg import Point

def clickCallback(data):
    print (data)
    print (camera.ptz.center(data.x, data.y))

rospy.init_node('axis_ptz')

axis_url = rospy.get_param('~url')

camera = axis_tracker.axis.Axis(axis_url)

rospy.Subscriber('click', Point, clickCallback)

rospy.spin()

