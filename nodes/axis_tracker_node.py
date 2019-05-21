#!/usr/bin/env python

import rospy
import axis_tracker.axis
import project11
import math

from geographic_msgs.msg import GeoPoint
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from axis_tracker.cfg import axis_trackerConfig

from dynamic_reconfigure.server import Server
from marine_traffic_com.cfg import marine_traffic_comConfig

zoom_level = 1.0
tilt_nudge = 0.0
heading_nudge = 0.0

basePosition = None
mode = 'look_at_target'

vehiclePositionBuffer = None

def modeCallback(data):
    global mode
    mode = data.data
    print data

def basePositionCallback(data):
    global basePosition
    basePosition = data.latitude, data.longitude
    
def lookAtCallback(data):
    global mode
    mode = 'look_at_target'
    doLookAt(data.latitude, data.longitude)
        
def vehiclePositionCallback(data):
    global vehiclePositionBuffer
    if vehiclePositionBuffer is None:
        vehiclePositionBuffer = (data.position.latitude,data.position.longitude)

    vehiclePositionBuffer = (data.position.latitude*.5 + vehiclePositionBuffer[0]*.5, data.position.longitude*0.5 + vehiclePositionBuffer[1]*.5)
    if mode == 'follow_vehicle':
        print vehiclePositionBuffer
        doLookAt(vehiclePositionBuffer[0],vehiclePositionBuffer[1])

def doLookAt(lat,lon):
    if basePosition is not None:
        azimuth, distance = project11.geodesic.inverse(math.radians(basePosition[1]),math.radians(basePosition[0]),
                                                       math.radians(lon),math.radians(lat))
        azimuth_degs = math.degrees(azimuth)
        if azimuth_degs > 180:
            azimuth_degs -= 360.0
        #print azimuth_degs, distance
        
        if camera_height > 0:
            tilt = math.degrees(math.atan(distance/camera_height))-90.0
            #print tilt
        else:
            tilt = 0.0

        camera.ptz.goto(azimuth_degs+heading_nudge,tilt+tilt_nudge,zoom_level,50)
    
def reconfigure_callback(config, level):
    global zoom_level
    global tilt_nudge
    global heading_nudge
    zoom_level = config['zoom']
    tilt_nudge = config['tilt_nudge']
    heading_nudge = config['heading_nudge']
    return config


rospy.init_node('axis_tracker')

axis_url = rospy.get_param('/base/camera/url')
axis_username = rospy.get_param('/base/camera/username')
axis_password = rospy.get_param('/base/camera/password')

camera = axis_tracker.axis.Axis(axis_url,axis_username,axis_password)

b_lat = rospy.get_param('/base/latitude', None)
b_long = rospy.get_param('/base/longitude', None)
if b_lat is not None and b_long is not None:
    basePosition = (b_lat, b_long)
    
camera_height = rospy.get_param('/base/camera/height',0.0)

rospy.Subscriber('/base/position', NavSatFix, basePositionCallback)
rospy.Subscriber('/base/camera/look_at', GeoPoint, lookAtCallback)
rospy.Subscriber('/base/camera/look_at_mode', String, modeCallback)

rospy.Subscriber('/udp/position', GeoPointStamped, vehiclePositionCallback)

config_server = Server(axis_trackerConfig, reconfigure_callback)


rospy.spin()
