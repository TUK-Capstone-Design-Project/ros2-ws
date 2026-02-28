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
        self.media_ros = Media_ROS()
        self.hand_detector = HandDetector()
        self.arm_status = True
        self.locking = True
        self.init = True
        self.pTime = 0
        self.add_lock = self.remove_lock = 0
        self.event = threading.Event()
        self.event.set()
        

    def process(self, frame):
        frame = cv.flip(frame, 1)
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        if len(lmList) != 0:
            threading.Thread(target=self.car_ctrl_threading, args=(lmList,bbox)).start()
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.media_ros.pub_imgMsg(frame)
        return frame

    def go_quadrilateral(self):

        # Move forward
        self.media_ros.pub_vel(0.15, 0.0, 0.0)
        sleep(2)
        self.media_ros.pub_vel(0.0, 0.3, 0.0)
        sleep(1)

        # Turn
        self.media_ros.pub_vel(-0.15, 0.0, 0.0)
        sleep(2)
        self.media_ros.pub_vel(0.0, -0.3, 0.0)
        sleep(1)
        
        self.media_ros.RobotBuzzer()
        sleep(1)

    def go_s(self):
        self.media_ros.pub_vel(0.0, 0.0,-1.0)
        sleep(0.5)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(0.5)
        self.media_ros.pub_vel(0.15, 0.0,0.0)
        sleep(1.5)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(0.5)

        self.media_ros.pub_vel(0.0, 0.0,2.0)
        sleep(0.5)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(0.5)
        self.media_ros.pub_vel(0.15, 0.0,0.0)
        sleep(1.5)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(0.5)

        self.media_ros.pub_vel(0.0, 0.0,-1.0)
        sleep(0.5)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(0.5)
        self.media_ros.pub_vel(0.15, 0.0,0.0)
        sleep(1.5)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(0.5)

        self.media_ros.pub_vel(0.0, 0.0,2.0)
        sleep(0.5)
        self.media_ros.pub_vel(0.0, 0.0,0.0)
        sleep(0.5)
        self.media_ros.pub_vel(0.15, 0.0,0.0)
        sleep(1.5)
        self.media_ros.RobotBuzzer()
        sleep(1)

    def Go_circle(self,flag):
        if (flag == 1):
            self.media_ros.pub_vel(0.1, 0.0,1.0)
            sleep(4)
        if (flag != 1):
            self.media_ros.pub_vel(0.1, 0.0,-1.0)
            sleep(4)
        self.media_ros.RobotBuzzer()
        sleep(1)


    def car_ctrl_threading(self, lmList,bbox):
        if self.event.is_set():
            self.event.clear()
            fingers = self.hand_detector.fingersUp(lmList)
            gesture = self.hand_detector.get_gesture(lmList)
            if gesture == "Yes":
                print("YES")	
                self.go_quadrilateral()
                sleep(3)

            elif gesture == "OK":
                print("OK")
                self.Go_circle(1)
                sleep(3)

            elif gesture == "Thumb_down":
                print("Thumb_down")
                self.media_ros.pub_vel(0.1, 0.0,0.0)
                sleep(2)
                self.media_ros.pub_vel(-0.1, 0.0,0.0)
                sleep(2)
                self.media_ros.RobotBuzzer()
                sleep(3)


            elif fingers[1] == fingers[4] == 1 and sum(fingers) == 2:
                print("ROCK")
                self.go_s()
                sleep(3)

            elif sum(fingers) == 5: 
                print("5")
                self.media_ros.pub_vel(0.0, 0.0,0.0)
                sleep(3)

            self.event.set()

def main():
    rclpy.init()
    pose_ctrl = PoseCtrlArm('posectrlarm')
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    while capture.isOpened():
        ret, frame = capture.read()
        frame = pose_ctrl.process(frame)
        if cv.waitKey(1) & 0xFF == ord('q'): 
            break
        cv.imshow('frame', frame)
    capture.release()
    cv.destroyAllWindows()
    rclpy.spin(pose_ctrl)
