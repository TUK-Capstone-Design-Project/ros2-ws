#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool, Int32MultiArray,Float32
from yahboomcar_msgs.msg import ServoControl
from Raspbot_Lib import Raspbot, LightShow
import time
class YahboomCarDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.car = Raspbot()
        self.car.Ctrl_Ulatist_Switch(1)
        self.light_show = LightShow()
        # Create subscribers
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.sub_RGBLight = self.create_subscription(Int32MultiArray, "rgblight", self.RGBLightcallback, 100)
        self.sub_BUzzer = self.create_subscription(Bool, "buzzer", self.Buzzercallback, 100)
        self.sub_servo = self.create_subscription(ServoControl, "servo", self.servo_callback, 10)

        # Create publishers
        self.pub_line_sensor = self.create_publisher(Int32MultiArray, "line_sensor", 10)
        self.pub_ultrasonic = self.create_publisher(Float32, "ultrasonic", 10)  
        # self.pub_all_sensors = self.create_publisher(Int32MultiArray, "all_sensors", 10)

        # Timer to publish all sensor data
        self.timer = self.create_timer(0.1, self.pub_data)

    def cmd_vel_callback(self, msg):
        if not isinstance(msg, Twist): return
        # Issue linear vel and angular vel
        vx = msg.linear.x * 1.0
        vy = msg.linear.y * 1.0
        vz = msg.angular.z * 1.0

        speed_lr = -vy * 255
        speed_fb = vx * 255
        speed_spin = -vz * (117+132)/8

        r1 = speed_fb + speed_lr + speed_spin
        r2 = speed_fb - speed_lr + speed_spin
        r3 = speed_fb - speed_lr - speed_spin
        r4 = speed_fb + speed_lr - speed_spin

        self.car.Ctrl_Muto(0, int(r1))
        self.car.Ctrl_Muto(1, int(r2))
        self.car.Ctrl_Muto(2, int(r3))
        self.car.Ctrl_Muto(3, int(r4))

    def RGBLightcallback(self, msg):
        # RGBLight control
        if not isinstance(msg, Int32MultiArray): return
        if len(msg.data) != 3:
            self.get_logger().warn("Invalid RGBLight data length. \'data: [0, 0, 0]\'. Turning off lights.")
            self.car.Ctrl_WQ2812_brightness_ALL(0, 0, 0)
        # Execute a specific light effect based on the message value
        R, G, B = msg.data
        self.car.Ctrl_WQ2812_brightness_ALL(R, G, B)

    def Buzzercallback(self, msg):
        if not isinstance(msg, Bool): return
        if msg.data:
            self.car.Ctrl_BEEP_Switch(1)
        else:
            self.car.Ctrl_BEEP_Switch(0)

    def servo_callback(self, msg):
        # Control both servos based on the received message
        if not isinstance(msg, ServoControl): return
        self.car.Ctrl_Servo(1, msg.servo_s1)
        self.car.Ctrl_Servo(2, msg.servo_s2)

    def pub_data(self):
        # Read and publish all sensor data
        # Example: Read line sensor data
        track = self.car.read_data_array(0x0a, 1)
        track = int(track[0])
        x1 = (track >> 3) & 0x01
        x2 = (track >> 2) & 0x01
        x3 = (track >> 1) & 0x01
        x4 = track & 0x01

        # Example: Add more sensors here
        # sensor2_data = self.read_sensor2_data()
        # sensor3_data = self.read_sensor3_data()

        linesensor_msg = Int32MultiArray(data=[x2, x1, x3, x4])
        self.pub_line_sensor.publish(linesensor_msg)

        diss_H = self.car.read_data_array(0x1b, 1)[0]
        diss_L = self.car.read_data_array(0x1a, 1)[0]
        dis = diss_H << 8 | diss_L  # 计算距离值
        ultrasonic_msg = Float32()
        ultrasonic_msg.data = float(dis/10)
        self.pub_ultrasonic.publish(ultrasonic_msg)

        # 添加日志输出
        #self.get_logger().info(f'Published line sensor data: {sensor_data.data}')

        

def main(args=None):
    rclpy.init(args=args)
    driver = YahboomCarDriver('driver_node')
    driver.get_logger().info(f'Successfully started the chassis drive...')
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.car.Ctrl_Ulatist_Switch(0)
        time.sleep(0.1)
        driver.car.Ctrl_Car(0, 0, 0)
        driver.car.Ctrl_Car(1, 0, 0)
        driver.car.Ctrl_Car(2, 0, 0)
        driver.car.Ctrl_Car(3, 0, 0)
        driver.get_logger().info(f'Ulatist off')
        driver.car.Ctrl_BEEP_Switch(0)
        driver.car.Ctrl_WQ2812_brightness_ALL(0, 0, 0)
        pass

""" if __name__ == '__main__':
    main() """