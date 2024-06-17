#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from robot_hand_control.msg import Hand

from GestrueController import *
   


hand = GestureController()
hand.start()

# last_pos = [1000, 1900, 1900, 1900, 1900]       # initial pos of finger
# motor = [1, 2, 3, 4, 5]

# # start the hand control
# hand_control = HandController('/dev/ttyUSB0')
# # hand position init
# hand_control.write(CMD_MULT_SERVO_MOVE, [85, 85, 20, 3, 5, 32, 3, 1, 208, 7, 2, 132, 3, 3, 132, 3, 4, 132, 3, 5, 132, 3])
# fingerRange = 1000      # from 900 (hold) to 1900 (lose), 
#                         # but for thumb, 2000 (hold) to 1000 (lose)

def sender():
    rospy.loginfo("Sending Data...")

    hand = GestureController()
    hand.start()

    # publisher
    pub = rospy.Publisher('hand', Hand, queue_size=10)
    rospy.init_node("hand_data_node")

    rate = rospy.Rate(24)

    while not rospy.is_shutdown():
        hand.on_frame()
        hand_controller_flag = hand.left_hand_controller
        movement_flag = hand.movement_flag

        # print(hand_controller_flag)

        if hand_controller_flag == True:
            # print("System Activated!")
            if movement_flag == True:
                ### extract hand data
                palm_position = hand.palm_position
                # print("Palm Position: ", palm_position)
                ypr = hand.ypr
                # print("YPR: ", ypr)

                hand_data = hand.hand_data
                finger_angles = hand.finger_angles
                mode = "Activated"

                msg = Hand()
                msg.palm_position = [hand_data[0], hand_data[1], hand_data[2]]
                msg.palm_normal = [hand_data[3], hand_data[4], hand_data[5]]
                msg.hand_direction = [hand_data[6], hand_data[7], hand_data[8]]
                msg.ypr = [hand_data[24], hand_data[25], hand_data[26]]
                msg.thumb_tip = [hand_data[9], hand_data[10], hand_data[11]]
                msg.index_tip = [hand_data[12], hand_data[13], hand_data[14]]
                msg.middle_tip = [hand_data[15], hand_data[16], hand_data[17]]
                msg.ring_tip = [hand_data[18], hand_data[19], hand_data[20]]
                msg.pinky_tip = [hand_data[21], hand_data[22], hand_data[23]]
                msg.finger_angles = finger_angles
                msg.mode = mode

                pub.publish(msg)
                rospy.loginfo(msg)
                rate.sleep()

            else:
                msg = Hand()
                msg.palm_position = [hand_data[0], hand_data[1], hand_data[2]]
                msg.palm_normal = [hand_data[3], hand_data[4], hand_data[5]]
                msg.hand_direction = [hand_data[6], hand_data[7], hand_data[8]]
                msg.ypr = [hand_data[24], hand_data[25], hand_data[26]]
                msg.thumb_tip = [hand_data[9], hand_data[10], hand_data[11]]
                msg.index_tip = [hand_data[12], hand_data[13], hand_data[14]]
                msg.middle_tip = [hand_data[15], hand_data[16], hand_data[17]]
                msg.ring_tip = [hand_data[18], hand_data[19], hand_data[20]]
                msg.pinky_tip = [hand_data[21], hand_data[22], hand_data[23]]
                msg.finger_angles = finger_angles
                msg.mode = "Freeze"

                pub.publish(msg)
                rospy.loginfo(msg)
                rate.sleep()

        else:
            # print("System Hold!")
            # hand_control.set_servo_position(motor, last_pos, 200)
            hand_data = hand.hand_data
            finger_angles = hand.finger_angles
            mode = False

            msg = Hand()
            msg.palm_position = [hand_data[0], hand_data[1], hand_data[2]]
            msg.palm_normal = [hand_data[3], hand_data[4], hand_data[5]]
            msg.hand_direction = [hand_data[6], hand_data[7], hand_data[8]]
            msg.ypr = [hand_data[24], hand_data[25], hand_data[26]]
            msg.thumb_tip = [hand_data[9], hand_data[10], hand_data[11]]
            msg.index_tip = [hand_data[12], hand_data[13], hand_data[14]]
            msg.middle_tip = [hand_data[15], hand_data[16], hand_data[17]]
            msg.ring_tip = [hand_data[18], hand_data[19], hand_data[20]]
            msg.pinky_tip = [hand_data[21], hand_data[22], hand_data[23]]
            msg.finger_angles = finger_angles
            msg.mode = "Freeze"

            pub.publish(msg)
            rospy.loginfo(msg)
            rate.sleep()

        

if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting...")