#!/usr/bin/env python3
# encoding: utf-8
import threading
import cv2 as cv
import numpy as np
from yahboomcar_mediapipe.media_library import *
from time import sleep, time
import rclpy
from rclpy.node import Node


class PoseCtrlArm(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub_servo = self.create_publisher(ServoControl, "servo", 10)
        self.x = 90
        self.y = 50
        self.servo_msg = ServoControl()  # 创建ServoControl消息实例
        self.servo_msg.servo_s1 = self.x  # 设置servo_s1
        self.servo_msg.servo_s2 = self.y  # 设置servo_s2
        self.pub_servo.publish(self.servo_msg)  # 发布ServoControl消息
        self.car_status = True
        self.stop_status = 0
        self.locking = False
        self.pose_detector = Holistic()
        self.hand_detector = HandDetector()
        self.pTime = self.index = 0
        self.media_ros = Media_ROS()
        self.event = threading.Event()
        self.event.set()

    def process(self, frame):
        frame = cv.flip(frame, 1)
        frame, lmList, _ = self.hand_detector.findHands(frame)
        if len(lmList) != 0:
            threading.Thread(target=self.hand_threading, args=(lmList,)).start()
        else:self.media_ros.pub_vel(0.0, 0.0,0.0)
            
        self.cTime = time()  
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime 
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.media_ros.pub_imgMsg(frame)
        return frame

    def hand_threading(self, lmList):
        if self.event.is_set():
            self.event.clear()
            self.stop_status = 0
            self.index = 0
            fingers = self.hand_detector.fingersUp(lmList)
            print("fingers: ", fingers)
            if sum(fingers) == 5: 
                self.media_ros.pub_vel(0.15, 0.0,0.0)
                sleep(0.5)
                
            elif sum(fingers) == 0: 
                self.media_ros.pub_vel(-0.15, 0.0,0.0)
                sleep(0.5)
                
            elif sum(fingers) == 1 and fingers[1] == 1: 
                self.media_ros.pub_vel(0.0, 0.0, 0.5)
                sleep(0.5)
                
            elif sum(fingers) == 2 and fingers[1] == 1 and fingers[2] == 1: 
                self.media_ros.pub_vel(0.0, 0.0, -0.5)
                sleep(0.5)
            else:
                self.media_ros.pub_vel(0.0, 0.0, 0.0)
            self.event.set()

    

def main():
    rclpy.init()
    pose_ctrl_arm = PoseCtrlArm('posectrlarm')
    capture = cv.VideoCapture(0)
   # capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    while capture.isOpened():
        ret, frame = capture.read()
        frame = pose_ctrl_arm.process(frame)
        if cv.waitKey(1) & 0xFF == ord('q'): break
        cv.imshow('frame', frame)
    capture.release()
    cv.destroyAllWindows()
    rclpy.spin(pose_ctrl_arm)
