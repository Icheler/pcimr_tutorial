#!/usr/bin/env python

import sys

import numpy as np

import roslib
import rospy

from pcimr_simulation.srv import InitPos
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

scan_data = 11
pos_data = 11

def callback(data, call):
    #laserscan
    if (call):
        global pos_data
        pos_data = [data.x,data.y]

    #position
    else:
        global scan_data
        scan_data = data.ranges



#service call to init_pos
rospy.wait_for_service('/init_pos')
init_pos=rospy.ServiceProxy('/init_pos', InitPos)
init_pos(2, 0)

#subscriber to scan data
scan = rospy.Subscriber('/scan', LaserScan, callback, 0)

#subscriber to robot position
pos = rospy.Subscriber('/robot_pos', Point, callback, 1)

#publisher mover
pub = rospy.Publisher('move', String, queue_size= 10)

rospy.init_node('mover')

subrate = rospy.Rate(1)
pubrate = rospy.Rate(0.5)

while not rospy.is_shutdown():
    subrate.sleep()
    print(pos_data)
    print(scan_data)
    if(scan_data[2]>1.0):
        pub.publish('N')
    elif(scan_data[3]>1.0):
        pub.publish('E')
    else:
        print('We should be done')
