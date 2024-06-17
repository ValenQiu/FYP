#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import math

# import for LeapMotion Camera
import threading
import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

# import for Sender
import argparse

import rospy
from leap_motion.msg import leap
from leap_motion.msg import leapros

# import for hand data
import pandas as pd
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt



class LeapFinger():
    def __init__(self, finger=None):
        self.boneNames = ['metacarpal',
                          'proximal',
                          'intermediate',
                          'distal']
        for boneName in self.boneNames:
            setattr(self, boneName, [0.0, 0.0, 0.0])
        self.tip = [0.0, 0.0, 0.0]

        self.leapBoneNames = [Leap.Bone.TYPE_METACARPAL,
                              Leap.Bone.TYPE_PROXIMAL,
                              Leap.Bone.TYPE_INTERMEDIATE,
                              Leap.Bone.TYPE_DISTAL]

        if finger is not None:
            self.importFinger(finger)

    def importFinger(self, finger):
        for boneName in self.boneNames:
            # Get the base of each bone
            bone = finger.bone(getattr(Leap.Bone, 'TYPE_%s' % boneName.upper()))
            setattr(self, boneName, bone.prev_joint.to_float_array())
        # For the tip, get the end of the distal bone
        self.tip = finger.bone(Leap.Bone.TYPE_DISTAL).next_joint.to_float_array()


class GestureController():
    def start(self):
        self.controller = Leap.Controller()
        ####### Define Variables #######
        # Hand capture flags
        self.left_hand_flag = False
        self.right_hand_flag = False
        self.movement_flag = False
        self.left_hand_controller = False

        # buffers
        # self.buff_l = [[], [], [], [], [], [], []]
        # self.buff_r = [[],[],[],[],[],[],[],[]]

        # data collection, if length of elements are longer than 20, then pop the first element, and append one new data in
        self.right_hand = [[] for _ in range(27)]   
            # average data from five frames
            # each three array save the x, y, z value of right hand parameters
            # palm_position, palm_normal, hand_direction, fingertips: thumb, index, middle, ring, pinky, ypr
        self.left_hand = [[] for _ in range(19)]    
            # average data from five frames
            # the first array save the grab_strength value [0, 1]
            # the rest value, each three array save the x, y, z value of right hand parameters
            # grab_stength, palm_position, fingertips: thumb, index, middle, ring, pinky
        self.hand_data = [[] for _ in range(27)]    
        # filtered data from the right_hand after passing through Gaussian_filter
        self.finger_vector = [[0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0]] 
        
        # For command
        self.finger_angles = [0, 0, 0, 0, 0]
        self.palm_position = [0, 0, 0]
        self.ypr = [0, 0, 0]    # pitch yaw roll
        self.last_pos = [1000, 1900, 1900, 1900, 1900]       # initial pos of finger
        
        print("Initialized Leap Motion Device!")

    def averaging_buff(self, points):
        total_points = len(points)
        if total_points == 0:
            return None
        
        avg_x = sum(point[0] for point in points) / len(points)
        avg_y = sum(point[1] for point in points) / len(points)
        avg_z = sum(point[2] for point in points) / len(points)

        return [avg_x, avg_y, avg_z]

    def cal_finger_angle(self, finger_vector, palm_normal):
        # two input vectors are unit vectors
        dot_product = finger_vector[0] * palm_normal[0] + finger_vector[1] * palm_normal[1] + finger_vector[2] * palm_normal[2]
        angle_radians = math.acos(dot_product)

        return angle_radians
    
    def conver_hand_target_position(self, finger_angles, fingerRange=1000):
        pos = [1000, 1900, 1900, 1900, 1900]
        for i in range(0,5):
            delta_pos = fingerRange * math.cos(finger_angles[i])
            if i == 0:
                pos[i] = int(pos[i] + delta_pos)
                if pos[i] >= 2000:
                    pos[i] = 2000
                elif pos[i] <= 1000:
                    pos[i] = 1000
            else:
                pos[i] = int(pos[i] - delta_pos)
                if pos[i] >= 1900:
                    pos[i] = 1900
                elif pos[i] <= 900:
                    pos[i] = 900
        return pos
    
    
    def on_frame(self):
        if(self.controller.is_connected):
            frame = self.controller.frame()
            buff_l = [[], [], [], [], [], [], [], []]
            # grab_strength, palm_position, thumb, index, middle, ring, pinky
            buff_r = [[],[],[],[],[],[],[],[], []]
            # palm_position, palm_normal, hand_direction, thumb, index, middle, ring, pinky, ypw
            left_hand_count = 0
            if frame.id % 5 == 0:
                frames = [frame, self.controller.frame(1), self.controller.frame(2), 
                          self.controller.frame(3), self.controller.frame(4)]
                
                for frame in frames:
                    for hand in frame.hands:
                        if hand.is_left:
                            self.left_hand_flag = True
                            self.left_hand_controller = True
                            left_hand_count += 1
                            # print("Left hand detected!")
                            self.right_hand_flag = False
                        elif hand.is_right:
                            self.right_hand_flag = True
                            # print("Right hand detected!")
                            self.left_hand_flag = False
                        else:
                            self.left_hand_controller = False

                        if self.left_hand_flag == True:       
                            grab_strength = hand.grab_strength
                            buff_l[0].append(grab_strength)
                            # If there is a left hand, then add the corresponding data to the buffer
                            palm_position = hand.palm_position
                            buff_l[1].append(palm_position)
                            fingers = hand.fingers

                            for finger in fingers:
                                fingertip = finger.tip_position
                                finger_type = finger.type
                                if finger_type == Leap.Finger.TYPE_THUMB:
                                    buff_l[2].append(fingertip)
                                elif finger_type == Leap.Finger.TYPE_INDEX:
                                    buff_l[3].append(fingertip)
                                elif finger_type == Leap.Finger.TYPE_MIDDLE:
                                    buff_l[4].append(fingertip)
                                elif finger_type == Leap.Finger.TYPE_RING:
                                    buff_l[5].append(fingertip)
                                elif finger_type == Leap.Finger.TYPE_PINKY:
                                    buff_l[6].append(fingertip)
                            # grab_strength, palm_position, thumb, index, middle, ring, pinky
                            
                            self.left_hand_flag = False

                        if self.right_hand_flag == True:
                             # If there is a right hand, then add the corresponding data to the buffer
                            palm_position = hand.palm_position
                            buff_r[0].append(palm_position)
                            palm_normal = hand.palm_normal
                            buff_r[1].append(palm_normal)
                            hand_direction = hand.direction
                            buff_r[2].append(hand_direction)
                            fingers = hand.fingers

                            for finger in fingers:
                                fingertip = finger.tip_position
                                finger_type = finger.type
                                if finger_type == Leap.Finger.TYPE_THUMB:
                                    buff_r[3].append(fingertip)
                                elif finger_type == Leap.Finger.TYPE_INDEX:
                                    buff_r[4].append(fingertip)
                                elif finger_type == Leap.Finger.TYPE_MIDDLE:
                                    buff_r[5].append(fingertip)
                                elif finger_type == Leap.Finger.TYPE_RING:
                                    buff_r[6].append(fingertip)
                                elif finger_type == Leap.Finger.TYPE_PINKY:
                                    buff_r[7].append(fingertip)
                            pitch = hand_direction.pitch * Leap.RAD_TO_DEG
                            yaw = palm_normal.yaw * Leap.RAD_TO_DEG
                            roll = hand_direction.roll * Leap.RAD_TO_DEG
                            ypr = [pitch, yaw, roll]
                            buff_r[8].append(ypr)
                            # print(ypr)
                            
                            self.right_hand_flag = False
                
                # check whether there is left hand in the frame
                if left_hand_count == 0:
                    self.left_hand_controller = False
            
                left_hand_count = 0
                
                for i in range(0, len(buff_l)):
                    if buff_l[i] != []:
                        if i == 0:
                            self.left_hand[0] = sum(buff_l[0]) /len(buff_l[0])
                        else:
                            buff_l[i] = self.averaging_buff(buff_l[i])
                            # check the length of the averageing buffer, if it is longer than 20, then pop the first element
                            if len(self.left_hand[i]) <= 3:
                                self.left_hand[i*3-2].append(buff_l[i][0])
                                self.left_hand[i*3-1].append(buff_l[i][1])
                                self.left_hand[i*3].append(buff_l[i][2])
                            else:
                                self.left_hand[i*3-2].pop(0)
                                self.left_hand[i*3-1].pop(0)
                                self.left_hand[i*3].pop(0)
                                self.left_hand[i*3-2].append(buff_l[i][0])
                                self.left_hand[i*3-1].append(buff_l[i][1])
                                self.left_hand[i*3].append(buff_l[i][2])

                        left_hand_trigger = self.left_hand[0]

                    buff_l[i] = []    # empty the coorespending buffer element.

                for i in range(0, len(buff_r)):
                    if buff_r[i] != []:
                        buff_r[i] = self.averaging_buff(buff_r[i])
                        # check the length of the averageing buffer, if it is longer than 20, then pop the first element
                        if len(self.right_hand[i]) <= 3:
                            self.right_hand[i*3].append(buff_r[i][0])
                            self.right_hand[i*3+1].append(buff_r[i][1])
                            self.right_hand[i*3+2].append(buff_r[i][2])
                        else:
                            self.right_hand[i*3].pop(0)
                            self.right_hand[i*3+1].pop(0)
                            self.right_hand[i*3+2].pop(0)
                            self.right_hand[i*3].append(buff_r[i][0])
                            self.right_hand[i*3+1].append(buff_r[i][1])
                            self.right_hand[i*3+2].append(buff_r[i][2])
                
                    buff_r[i] = []    # empty the coorespending buffer element.
                
                # print("Left Hand: ", self.left_hand)
                # print("Right Hand: ", self.right_hand)

                if self.left_hand_controller == True:
                    left_hand_trigger = self.left_hand[0]
                    if left_hand_trigger != []:
                        if left_hand_trigger > 0.6:
                            self.movement_flag = True
                        else:
                            self.movement_flag = False

                    if self.movement_flag == True:
                        # print("Acturating the system!")
                        if self.right_hand[0] != []:
                            # 1D Gaussian Filter
                            if len(self.right_hand[0]) > 3:
                                for i in range(0, len(self.hand_data)):
                                    data = gaussian_filter1d(self.right_hand[i], 1)
                                    length = len(data)-1
                                    self.hand_data[i] = data[length]

                            # print(self.hand_data)

                            # Calculate Finger Vector
                            for i in range(0, 5):
                                x = self.hand_data[3*i + 9] - self.hand_data[0]
                                y = self.hand_data[3*i + 10] - self.hand_data[1]
                                z = self.hand_data[3*i + 11] - self.hand_data[2]
                                abs = math.sqrt(x**2 + y**2 + z**2)
                                self.finger_vector[i] = [x/ abs, y/ abs, z/ abs] 

                            # print(finger_vector)

                            # Calculate Finger Angles
                            palm_normal = [self.hand_data[3], self.hand_data[4], self.hand_data[5]]

                            for i in range(0, 5):
                                if self.finger_vector[i] != []:
                                    self.finger_angles[i] = self.cal_finger_angle(self.finger_vector[i], palm_normal)
                            
                            # print(self.finger_angles)

                            # Calculate Palm Position
                            palm_position = [self.hand_data[0], self.hand_data[1], self.hand_data[2]]
                            self.palm_position = palm_position
                            ypr = [self.hand_data[24], self.hand_data[25], self.hand_data[26]]
                            self.ypr = ypr

                            ############# TO DO Calculate the pitch yaw roll ####################

                    
                    else:
                        # print("System Hold!")
                        pass
                
                else:
                    self.movement_flag == False
                    # print("No left hand!")