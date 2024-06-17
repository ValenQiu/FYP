#!/usr/bin/env python
#coding=utf-8

import rospy
import rospy
from std_msgs.msg import String
from robot_hand_control.msg import Hand

import math
import numpy as np
import time

# import hand controller
from HandController import HandController
CMD_MULT_SERVO_MOVE = 3
CMD_FULL_ACTION_STOP = 8
CMD_SERVO_MOVE_STOP = 8

finger_angles = np.array([0, 0, 0, 0, 0])
mode = "Freeze"

# initialize the GestureController
last_pos = [1000, 1900, 1900, 1900, 1900]       # initial pos of finger
motor = [1, 2, 3, 4, 5]

pos = np.array([1000, 1900, 1900, 1900, 1900])

# start the hand control
hand_control = HandController('/dev/ttyUSB0')
# hand position init
hand_control.write(CMD_MULT_SERVO_MOVE, [85, 85, 20, 3, 5, 32, 3, 1, 208, 7, 2, 132, 3, 3, 132, 3, 4, 132, 3, 5, 132, 3])
fingerRange = 1000      # from 900 (hold) to 1900 (lose), 
                        # but for thumb, 2000 (hold) to 1000 (lose)


def conver_hand_target_position(finger_angles, fingerRange=1000):
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


def callback(msg):
    rospy.loginfo("Listenering...")
    finger_angles = msg.finger_angles
    mode = msg.mode
    rospy.loginfo(finger_angles)
    rospy.loginfo(mode)
    pos = conver_hand_target_position(finger_angles, fingerRange=1000)
    print(pos)
    last_pos = pos
    hand_control.set_servo_position(motor, pos, 200)
    

def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('hand', Hand, callback, queue_size=1)
    rospy.spin()




if __name__ == '__main__':
    
    try:
        listener()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting...")