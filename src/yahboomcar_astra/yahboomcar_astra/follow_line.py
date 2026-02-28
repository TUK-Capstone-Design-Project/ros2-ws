#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, Image
#common lib
import os
import threading
import math
from yahboomcar_astra.follow_common import *
from std_msgs.msg import Int32, Bool,UInt16
from yahboomcar_msgs.msg import *
from Raspbot_Lib import Raspbot

RAD2DEG = 180 / math.pi
print ("import finish")
cv_edition = cv.__version__
print("cv_edition: ",cv_edition)

class LineDetect(Node):
	def __init__(self,name):
		super().__init__(name)
		self.bot = Raspbot()
		#create a publisher
		self.pub_cmdVel = self.create_publisher(Twist,"/cmd_vel",1)
		self.pub_Buzzer = self.create_publisher(UInt16,"/beep",1)
		self.pub_servo = self.create_publisher(ServoControl, "servo", 10)
		self.x = 90
		self.y = 0
		self.servo_msg = ServoControl()  # 创建ServoControl消息实例
		self.servo_msg.servo_s1 = self.x  # 设置servo_s1
		self.servo_msg.servo_s2 = self.y  # 设置servo_s2
		self.pub_servo.publish(self.servo_msg)  # 发布ServoControl消息
	
		#create a subscriber
		self.img = None
		self.circle = ()
		self.hsv_range = ()
		self.Roi_init = ()   
		self.end = 0
		self.Start_state = True
		self.dyn_update = False
		self.Buzzer_state = False
		self.select_flags = False
		self.Track_state = 'identify'
		self.windows_name = 'frame'
		self.cols, self.rows = 0, 0
		self.Mouse_XY = (0, 0)
		self.hsv_text = "/root/yahboomcar_ws/src/yahboomcar_astra/yahboomcar_astra/colorHSV.text"
		self.color = color_follow()
		self.scale = 1000
		self.FollowLinePID = (60, 0, 20)
		self.linear = 0.2
		self.PID_init()	
		self.declare_param()
		self.img_flip = True
		self.refresh  = False
		self.capture = cv.VideoCapture(0)
		#if cv_edition[0]=='3': self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
		#else: self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
		self.timer = self.create_timer(0.001, self.on_timer)	

	def declare_param(self):
		#HSV
		self.declare_parameter("Hmin",0)
		self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
		self.declare_parameter("Smin",85)
		self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
		self.declare_parameter("Vmin",126)
		self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
		self.declare_parameter("Hmax",9)
		self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
		self.declare_parameter("Smax",253)
		self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
		self.declare_parameter("Vmax",253)
		self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
        #PID
		self.declare_parameter("Kp",60)
		self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
		self.declare_parameter("Ki",0)
		self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
		self.declare_parameter("Kd",20)
		self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
		#other
		self.declare_parameter("scale",1000)
		self.scale = self.get_parameter('scale').get_parameter_value().integer_value
		self.declare_parameter("linear",0.18)
		self.linear = self.get_parameter('linear').get_parameter_value().double_value
		self.declare_parameter('refresh',False)
		self.refresh = self.get_parameter('refresh').get_parameter_value().bool_value
		self.pub_servo.publish(self.servo_msg)  # 发布ServoControl消息
		
		

	def PID_init(self):
		self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[1] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[2] / 1.0 / (self.scale), 0])
		
	def onMouse(self, event, x, y, flags, param):
		if event == 1:
			self.Track_state = 'init'
			self.select_flags = True
			self.Mouse_XY = (x,y)
		if event == 4:
			self.select_flags = False
			self.Track_state = 'mouse'
		if self.select_flags == True:
			self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
			self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
			self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

	def execute(self, point_x, color_radius):
		self.Start_state = True
		if color_radius == 0: 
			print("Not Found")
			self.pub_cmdVel.publish(Twist())
		else:
			twist = Twist()
			b = UInt16()
			[z_Pid, _] = self.PID_controller.update([(point_x - 320)*1.0/16, 0])
			if self.img_flip == True: twist.angular.z = -z_Pid #-z_Pid
			else: twist.angular.z = +z_Pid
			#self.get_logger().info('center = {} angular ={}'.format([(point_x)],[twist.angular.z]))
			twist.linear.x = self.linear
			if self.Buzzer_state == True:
				b.data = 0
				for i in range(3): self.pub_Buzzer.publish(b)
				self.Buzzer_state = False
			
			if abs(point_x-320)<40:
				twist.angular.z=0.0
			self.pub_cmdVel.publish(twist)
			print(twist.linear.x,twist.angular.z)
                
						


	def process(self, rgb_img, action):
		
		binary = []
		rgb_img = cv.resize(rgb_img, (640, 480))
        
		if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
		if action == 32: 
			self.Track_state = 'tracking'
			print("tracking")
		elif action == ord('i') or action == 105: self.Track_state = "identify"
		elif action == ord('r') or action == 114: self.Reset()
		elif action == ord('q') or action == 113: self.cancel()
		if self.Track_state == 'init':
			cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
			cv.setMouseCallback(self.windows_name, self.onMouse, 0)
			if self.select_flags == True:
				cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
				cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
				if self.Roi_init[0]!=self.Roi_init[2] and self.Roi_init[1]!=self.Roi_init[3]:
					rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
					self.dyn_update = True
    
			else: 
					self.Track_state = 'init'
		elif self.Track_state == "identify":
			#print(self.circle[0])
			if os.path.exists(self.hsv_text): self.hsv_range = read_HSV(self.hsv_text)
			else: self.Track_state = 'init'
		if self.Track_state != 'init' and len(self.hsv_range) != 0:
			rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
			if self.dyn_update == True:
				write_HSV(self.hsv_text, self.hsv_range)
				self.Hmin  = rclpy.parameter.Parameter('Hmin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][0])
				self.Smin  = rclpy.parameter.Parameter('Smin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][1])
				self.Vmin  = rclpy.parameter.Parameter('Vmin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][2])
				self.Hmax  = rclpy.parameter.Parameter('Hmax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][0])
				self.Smax  = rclpy.parameter.Parameter('Smax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][1])
				self.Vmax  = rclpy.parameter.Parameter('Vmax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][2])

				
				all_new_parameters = [self.Hmin,self.Smin,self.Vmin,self.Hmax,self.Smax,self.Vmax]
				self.set_parameters(all_new_parameters)
				self.dyn_update = False
		if self.Track_state == 'tracking':
			if len(self.circle) != 0:
				threading.Thread(target=self.execute, args=(self.circle[0], self.circle[2])).start()
		else:
			if self.Start_state == True:
				#self.pub_cmdVel.publish(Twist())
				self.Start_state = False
		return rgb_img, binary


	def on_timer(self):
		self.get_param()
		ret, frame = self.capture.read()
		action = cv.waitKey(10) & 0xFF
		frame, binary =self.process(frame, action)
		start = time.time()
		fps = 1 / (start - self.end)
		text = "FPS : " + str(int(fps))
		self.end = start
		cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
		if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
		else:cv.imshow('frame', frame)
		if action == ord('q') or action == 113:
			self.capture.release()
			cv.destroyAllWindows()

	def Reset(self):
		self.PID_init()
		self.Track_state = 'init'
		self.hsv_range = ()
		self.Mouse_XY = (0, 0)
		self.pub_cmdVel.publish(Twist())
		print("Reset succes!!!")

	def get_param(self):
		#hsv
		self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
		self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
		self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
		self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
		self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
		self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
		#kpi
		self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
		self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
		self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
		self.FollowLinePID = (self.Kp,self.Ki,self.Kd)
		#
		self.scale = self.get_parameter('scale').get_parameter_value().integer_value
		self.linear = self.get_parameter('linear').get_parameter_value().double_value
		self.refresh = self.get_parameter('refresh').get_parameter_value().bool_value

	def cancel(self):
		self.bot.Ctrl_Servo(1, 90)
		self.bot.Ctrl_Servo(2, 25)
		self.capture.release()
		cv.destroyAllWindows()
		print ("Shutting down this node.")
		

	
def main(args=None):
	rclpy.init(args=args)
	linedetect = LineDetect("follow_line")
	try:
		print("start it")
		rclpy.spin(linedetect)
	except KeyboardInterrupt:
		linedetect.capture.release()
		linedetect.bot.Ctrl_Servo(1, 90)
		linedetect.bot.Ctrl_Servo(2, 25)
		pass

	
	
	

