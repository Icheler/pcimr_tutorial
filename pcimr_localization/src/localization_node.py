#!/usr/bin/env python

import sys

import numpy as np

import roslib
import rospy

from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker

MOVE_IDS = ['N', 'S', 'W', 'E']

MOVE = [np.array([0, 1]),     # Up
        np.array([0, -1]),    # Down
        np.array([-1, 0]),    # Left
        np.array([1, 0])]     # Right

def numpy_to_occupancy_grid(arr, info=None):
    if not len(arr.shape) == 2:
        raise TypeError('Array must be 2D')

    # Adjust values
    arr = arr.astype('float64')
    factor = 100/np.max(arr)
    arr[arr > 0] *= factor

    arr = np.array(np.transpose(arr), dtype=np.int8)

    grid = OccupancyGrid()
    if isinstance(arr, np.ma.MaskedArray):
        # We assume that the masked value are already -1, for speed
        arr = arr.data
    grid.data = arr.ravel()
    grid.info = info or MapMetaData()
    grid.info.height = arr.shape[0]
    grid.info.width = arr.shape[1]

    return grid

class LocalizationNode:
    def __init__(self):
        self.scan_data = None
        self.map_data = None
        self.map_msg = None
        self.move_data = None

        self.move_probs = rospy.get_param("robot_move_probabilities", [0.9, 0.04, 0.04, 0.0, 0.02])

        #subscriber 
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.cb_map)
        self.sub_move = rospy.Subscriber('/move', String, self.cb_move)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.cb_scan)

        #publisher
        self.pub_pos = rospy.Publisher('/robot_pos', Point, queue_size=10)
        #TODO
        self.pub_marker = rospy.Publisher('/visualization/robot_pos', Marker, queue_size=10)
        self.pub_posestimate = rospy.Publisher('/robot_pos_map', OccupancyGrid, queue_size=10)

        self.msg_pos = Point()

        self.msg_marker= Marker()
        self.msg_marker.header.frame_id = "map"
        self.msg_marker.ns = "navigation"
        self.msg_marker.id = 0
        self.msg_marker.type = Marker.CUBE
        self.msg_marker.action = Marker.ADD
        self.msg_marker.scale.x = 1
        self.msg_marker.scale.y = 1
        self.msg_marker.scale.z = 0.2
        self.msg_marker.color.a = 1.0
        self.msg_marker.color.r = 0.0
        self.msg_marker.color.g = 0.8
        self.msg_marker.color.b = 1.0
        self.msg_marker.pose.orientation = Quaternion(0, 0, 0, 1)

    def cb_map(self, msg):
        self.map_msg = msg
        self.map_data = np.transpose(np.asarray(msg.data, dtype=np.int8).reshape(msg.info.width, msg.info.height))

    def cb_move(self, msg):
        self.move_data = msg

    def cb_scan(self, msg):
        self.scan_data = msg.ranges

    def give_ranges(self, map):
        map_ranges = np.full((20, 20, 4), -1)
        for i in range(len(map)):
            for j in range(len(map[0])):
                if not (map[i][j] == -1 or map[i][j] == 100):
                    map_ranges[i][j] = self.give_range(map, i, j)
        return map_ranges

    def give_range(self, map, x, y):
        #north
        pos_range = np.full(4, -1)
        for i in range(x, -1, -1):
            if(map[i][y] == 100):
                pos_range[1]=x-i
                break
            if(i==0):
                pos_range[1]=x-i
                break
        #east
        for i in range(y, len(map[0])):
            if(map[x][i] == 100):
                pos_range[2] = i-y
                break
            if(i == len(map[0])):
                pos_range[2] = i-y
        #south
        for i in range(x, len(map)):
            if(map[i][y] == 100):
                pos_range[3]= i-x
                break
            if(i == len(map)-1):
                pos_range[3]= i-x
        #west
        for i in range(y, -1, -1):
            if(map[x][i] == 100):
                pos_range[0] = y-i
                break
            if(i == np.abs(y-len(map[0]))):
                pos_range[0] = y-i

            if(i == 0):
                break

        return pos_range

    def prob_scan_cell(self, scan_data, cell_ranges, cell_data):
        probs = np.zeros(4)
        for i in range(len(cell_ranges)):
            if(scan_data[i]==cell_ranges[i]):
                probs[i]=0.8
            elif(scan_data[i]-1==cell_ranges[i] or scan_data[i]+1==cell_ranges[i]):
                probs[i]=0.1
        return np.prod(probs)

    def prob_scan_all_cells(self, scan_data, map_ranges, map_data):
        scan_probs = np.full((20, 20), -1.)
        for i in range(len(map_ranges)):
            for j in range(len(map_ranges[0])):
                if not (-1 in map_ranges[i][j]):
                    scan_probs[i][j] = self.prob_scan_cell(scan_data, map_ranges[i][j], map_data[i][j])
                else:
                    scan_probs[i][j] = 0
        return scan_probs
    
    def cell_checker(self, map_data, x, y):
        try:
            if(map_data[x][y] is None or map_data[x][y] == -1 or map_data[x][y] == 100):
                return 0
            return 1
        except IndexError:
            return 0

    def prob_move_cell(self, move, map_data, x, y):
        probs = np.zeros(5)
        if (move == 'N'):
            if(self.cell_checker(map_data, x, y-1)):
                probs[0] = self.move_probs[0] * map_data[x, y-1]
            if(self.cell_checker(map_data, x+1, y)):
                probs[1] = self.move_probs[1] * map_data[x+1, y]
            if(self.cell_checker(map_data, x-1, y)):
                probs[2] = self.move_probs[2] * map_data[x-1, y]
            probs[3] = self.move_probs[3]
            probs[4] = self.move_probs[4] * map_data[x, y]
        elif (move == 'W'):
            if(self.cell_checker(map_data, x-1, y)):
                probs[0] = self.move_probs[0] * map_data[x-1, y]
            if(self.cell_checker(map_data, x, y+1)):
                probs[1] = self.move_probs[1] * map_data[x, y+1]
            if(self.cell_checker(map_data, x, y-1)):
                probs[2] = self.move_probs[2] * map_data[x, y-1]
            probs[3] = self.move_probs[3]
            probs[4] = self.move_probs[4] * map_data[x, y]
        elif (move == 'S'):
            if(self.cell_checker(map_data, x, y+1)):
                probs[0] = self.move_probs[0] * map_data[x, y+1]
            if(self.cell_checker(map_data, x+1, y)):
                probs[1] = self.move_probs[1] * map_data[x+1, y]
            if(self.cell_checker(map_data, x-1, y)):
                probs[2] = self.move_probs[2]*  map_data[x-1, y]
            probs[3] = self.move_probs[3]
            probs[4] = self.move_probs[4] * map_data[x, y]
        elif (move == 'E'):
            if(self.cell_checker(map_data, x+1, y)):
                probs[0] = self.move_probs[0] * map_data[x+1, y]
            if(self.cell_checker(map_data, x, y+1)):
                probs[1] = self.move_probs[1] * map_data[x, y+1]
            if(self.cell_checker(map_data, x, y-1)):
                probs[2] = self.move_probs[2] * map_data[x, y-1]
            probs[3] = self.move_probs[3] 
            probs[4] = self.move_probs[4] * map_data[x, y]
        else:
            print('Move command is not NWSE')

        return np.sum(probs)

    def prob_move_all_cells(self, move, map_data):
        move_probs = np.full((20, 20), 0.)
        for i in range(len(map_data)):
            for j in range(len(map_data[0])):
                if (self.cell_checker(map_data, i, j)):
                    move_probs[i][j] = self.prob_move_cell(move, map_data, i, j)
                else:
                    move_probs[i][j] = 0
        return move_probs

    def run(self, rate: float = 1):
        subrate = rospy.Rate(rate)

        while (self.map_data is None or self.scan_data is None) and not rospy.is_shutdown():
            subrate.sleep()

        last_map = None
        map_ranges = self.give_ranges(self.map_data)

        num_free_cells = np.sum(self.map_data == 0)
        prob_cell = 1/num_free_cells

        while not rospy.is_shutdown():
            local_move = self.move_data
            local_map = self.map_data.astype('float64')
            local_scan = self.scan_data

            if(local_move is None):
                local_map = np.where(local_map == 0, prob_cell, local_map)
            else:
                local_move = self.move_data.data
                local_map = local_map + last_map
                local_map = np.where(local_map == 0, 0.00001, local_map)

                local_map = self.prob_move_all_cells(local_move, local_map)

            map_scan_prob = self.prob_scan_all_cells(local_scan, map_ranges, local_map)

            local_map = local_map*map_scan_prob

            arr_sum = np.sum(local_map)
            local_map = np.where(local_map != 0, local_map/arr_sum, local_map)
            last_map = local_map
            
            #times 100 and round to nearest int
            local_map = local_map * 100
            local_map = np.round(local_map)

            pos_estimate_msg = numpy_to_occupancy_grid(local_map)

            self.pub_posestimate.publish(pos_estimate_msg)

            best_x, best_y = np.unravel_index(local_map.argmax(), local_map.shape)
            print(best_x, best_y)
            self.msg_marker.pose.position.x = best_x + 0.5
            self.msg_marker.pose.position.y = best_y + 0.5

            self.pub_marker.publish(self.msg_marker)

            self.msg_pos.x = best_x
            self.msg_pos.y = best_y

            self.pub_pos.publish(self.msg_pos)

            #prob for current position
            
            #probability for sensing

            #normalize

            #prints
            #rotate array back 

            subrate.sleep()


__name__ == "__main__"
rospy.init_node('localization_node')

localization_node = LocalizationNode()
localization_node.run(rate=1)