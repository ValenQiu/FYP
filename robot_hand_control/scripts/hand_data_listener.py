#!/usr/bin/env python
#coding=utf-8

import rospy
import rospy
from std_msgs.msg import String
from robot_hand_control.msg import Hand

def callback(msg):
    rospy.loginfo("Listenering...")
    rospy.loginfo(msg.hand_direction)
    rospy.loginfo(msg.palm_normal)
    rospy.loginfo(msg.palm_position)
    rospy.loginfo(msg.ypr)
    rospy.loginfo(msg.thumb_tip)
    rospy.loginfo(msg.index_tip)
    rospy.loginfo(msg.middle_tip)
    rospy.loginfo(msg.ring_tip)
    rospy.loginfo(msg.pinky_tip)
    rospy.loginfo(msg.mode)

def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('hand', Hand, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.logwarn("Start Listening")
    listener()