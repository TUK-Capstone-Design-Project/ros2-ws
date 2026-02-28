#!/usr/bin/env python3
# encoding: utf-8
import threading
import cv2 as cv
import numpy as np
from yahboomcar_mediapipe.media_library import *
from time import sleep, time
import rclpy
from rclpy.node import Node


class HandCtrlArm(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub_servo = self.create_publisher(ServoControl, "servo", 10)
        self.x = 90
        self.y = 50
        self.servo_msg = ServoControl()  # 创建ServoControl消息实例
        self.servo_msg.servo_s1 = self.x  # 设置servo_s1
        self.servo_msg.servo_s2 = self.y  # 设置servo_s2
        self.pub_servo.publish(self.servo_msg)  # 发布ServoControl消息
        self.media_ros=Media_ROS()
        self.hand_detector = HandDetector()
        self.arm_status = True
        self.locking = True
        self.init = True
        self.pTime = 0
        self.add_lock = self.remove_lock = 0
        self.event = threading.Event()
        self.event.set()

    def calculate_palm_center(self,bbox):
        xmin, ymin, xmax, ymax = bbox
        bbox_center_x = (xmin + xmax) // 2
        bbox_center_y = (ymin + ymax) // 2
        return bbox_center_x, bbox_center_y
    
    def process(self, frame):
        frame = cv.flip(frame, 1)
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        if len(lmList) != 0:
            threading.Thread(target=self.arm_ctrl_threading, args=(lmList,bbox)).start()
            point_x ,point_y= self.calculate_palm_center(bbox)
            cv.circle(frame, (point_x ,point_y), 5, (0, 0, 255), cv.FILLED)
        else:
            self.media_ros.pub_vel(0.0,0.0,0.0)
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.media_ros.pub_imgMsg(frame)
        return frame

    def arm_ctrl_threading(self, lmList,bbox):
        if self.event.is_set():
            self.event.clear()
            fingers = self.hand_detector.fingersUp(lmList)
            self.hand_detector.draw = True
            gesture = self.hand_detector.get_gesture(lmList)
            self.arm_status = False
            point_x ,point_y= self.calculate_palm_center(bbox)
            # point_x = lmList[9][1]
            # point_y = lmList[9][2]

            print("x y",point_x,point_y)
            if point_y >= 380: x = -0.15
            elif point_y <= 100: x = 0.15
            else: x = 0.0
            if point_x >= 440: y = -1.0
            elif point_x <= 200: y = 1.0
            else: y = 0.0
            if(x==0):
                if(y>0):y=0.15 
                elif(y<0):y=-0.15
                self.media_ros.pub_vel(x,y,0.0)
            else:self.media_ros.pub_vel(x,0.0,y)
            print("angle: {},value: {}".format(x,y))
            self.arm_status = True
            self.event.set()


def main():
    rclpy.init()
    handctrlarm = HandCtrlArm('handctrl')
    capture = cv.VideoCapture(0)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    while capture.isOpened():
        ret, frame = capture.read()
        action = cv.waitKey(1) & 0xFF
        frame = handctrlarm.process(frame)
        if action == ord('q'):
            handctrlarm.media_ros.cancel()
            break
        cv.imshow('frame', frame)
    capture.release()
    cv.destroyAllWindows()
    rclpy.spin(handctrlarm)
