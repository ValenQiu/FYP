#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import serial
import serial.tools.list_ports
from threading import Lock
import time

ports_list = list(serial.tools.list_ports.comports())  # 获取所有串口设备实例
if len(ports_list) <= 0:
    print("无可用的串口设备！")
else:
    print("可用的串口设备如下：")
    for port in ports_list:  # 依次输出每个设备对应的串口号和描述信息
        print(list(port)[0], list(port)[1])  # COM4 USB-SERIAL CH340 (COM4)


"""
# TO DO:
#     将命令符添加进这里
#     将CMD_SERVO_MOVE命令符替换成机械手用的
#     然后把后面用到CMD_SERVO_MOVE的变量名都改成对应的
#     （虽然数值都是3，但是跟着协议的名字用避免混乱）
# """
# CMD_SERVO_FRAME_HEADER = 0x55
# CMD_SERVO_FRAME_HEADER = 85 # pyserial 可以自动encode，

CMD_MULT_SERVO_MOVE = 3
CMD_FULL_ACTION_STOP = 8
CMD_SERVO_MOVE_STOP = 8


class HandController():

    def __init__(self, port):
        """open the port, initialization (open the serial port. Initialize the parameter)"""
        try:
            self.serial_mutex = Lock()
            self.ser = None
            self.timeout = 10
            self.ser = serial.Serial(port, 9600)
            if self.ser.isOpen():  # 判断串口是否成功打开
                print("串口成功打开")
                print(self.ser.name)  # 输出串口号，即COM4
            else:
                print("串口打开失败")
            self.port_name = port
        except:
            print("error")

    def __del__(self):
        self.close()

    def close(self):
        """
               close the serial port.
               """
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()
        if self.ser.isOpen():  # 判断串口是否关闭
            print("串口未关闭")
        else:
            print("串口已关闭")

    def __write_serial(self, data):
        # self.ser.flushInput()
        self.ser.write(data)
        time.sleep(0.00034)

    def write(self, cmd_type, params):
        # data received from "set_servo_position"
        length = 2 + len(params)  # length, cmd, params, checksum
            # print("length: ", length)
            # Check Sum = ~ ((ID + LENGTH + COMMAND + PARAM_1 + ... + PARAM_N) & 0xFF)
        packet = [85,85, length, cmd_type]
        for param in params:
            packet.append(param)
        # print("packet", packet)
        with self.serial_mutex:
            self.__write_serial(packet)

    def set_servo_position(self, motor_id, position, duration=None):
        '''
        驱动串口舵机转到指定位置(drive the serial servo to rotate to the designated position)
        :motor_id: the id of motor will be driven, Default: all 5
        :position: Position of 5 motor, type: list
        :duration: time that finish the operation, int
        '''
        # print("id:{}, pos:{}, duration:{}".format(hand_id, position, duration))

        current_timestamp = time.time()
        if duration is None:
            duration = 20
        duration = int(duration)
        position = [int(p) for p in position]
        # 角度高低八位
        loVal  = [int(p & 0xFF) for p in position]
        hiVal = [int(p >> 8) for p in position]
            # loVal = int(position & 0xFF)
            # hiVal = int(position >> 8)
        # 时间高低八位
        loTime = int(duration & 0xFF)
        hiTime = int(duration >> 8)
        # 第一位是控制舵机个数
        # header of motor will be defined in Write function
        motor_num = len(motor_id)
        params = [motor_num,loTime, hiTime]

        for i in motor_id:
            params.append(i)
            params.append(loVal[i-1])
            params.append(hiVal[i-1])
            # params = [motor_num,loTime, hiTime,1,loVal,hiVal,2,loVal,hiVal ...]

        self.write(CMD_MULT_SERVO_MOVE, params)

    def stop(self, hand_id):
        '''
        停止舵机运行(stop servo rotation)
        :param id:
        :return:
        '''
        self.write(hand_id, CMD_SERVO_MOVE_STOP, ())

    def exception_on_error(self, error_code, hand_id, command_failed):
        global exception
        exception = None

        if not isinstance(error_code, int):
            ex_message = '[servo #%d on %s@%sbps]: %s failed' % (
            hand_id, self.ser.port, self.ser.baudrate, command_failed)
            msg = 'Communcation Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, 0)
            return


class ChecksumError(Exception):
    def __init__(self, hand_id, response, checksum):
        Exception.__init__(self)
        self.message = 'Checksum received from motor %d does not match the expected one (%d != %d)' \
                       % (hand_id, response[-1], checksum)
        self.response_data = response
        self.expected_checksum = checksum

    def __str__(self):
        return self.message


class FatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message


class NonfatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message


class ErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message


class DroppedPacketError(Exception):
    def __init__(self, message):
        Exception.__init__(self)
        self.message = message

    def __str__(self):
        return self.message


# hand_control = HandController('/dev/ttyUSB0')

# hand_control.write(CMD_MULT_SERVO_MOVE, [85, 85, 20, 3, 5, 32, 3, 1, 208, 7, 2, 132, 3, 3, 132, 3, 4, 132, 3, 5, 132, 3])

# # test RoboHand 
# while True:
#     try:

#         motor = [1,2,3,4,5]
#         position = [1500]*5
#         pos = [[900, 900, 900, 900, 2000],
#                 [1200, 1200, 1200, 1200, 1700],
#                 [1500, 1500, 1500, 1500, 1400],
#                 [1900, 1900, 1900, 1900, 1000]]
        
#         for i in pos:
#             i.reverse()
#             hand_control.set_servo_position(motor,i,800)
#             if i == pos[-1]:
#                 time.sleep(4)
#             else:
#                 time.sleep(2)

#     except KeyboardInterrupt:      
#         # reset position of motor
#         hand_control.set_servo_position(motor,position,500)
#         sys.exit(0)
#         break






