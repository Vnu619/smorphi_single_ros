#!/usr/bin/env python3
import rospy
import time
import math
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import serial


#file1 = open("/home/wasp/Desktop/velocities.txt", "a")


class WASPMini(object):
	def __init__(self):
		self.wheel_odom_pub = rospy.Publisher('wheel_odom', Odometry, queue_size=50)
		rospy.Subscriber('/cmd_vel', Twist, self.writespeed)
		rospy.Subscriber('/imu_data', Imu, self.imucallback)
		rospy.Subscriber("/shape_need", Int32, self.shpneed)
		self.shape_pub = rospy.Publisher("current_shape", Int32, queue_size=50)
		self.ser = serial.Serial('/dev/ttyUSB2',115200, timeout=0.005)

		self.x = 0.0
		self.y = 0.0
		self.th = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.w = 0.0
		self.odomBroadcaster = tf.TransformBroadcaster()
		self.imu_msg = Imu()
		self.shape_need = 0
		self.cur_shape = 0
		self.xdot = 0
		self.thetadot = 0
		self.xdot1 = 0
		self.thetadot1 = 0
		self.roll = 0
		self.pitch = 0
		self.prev_yaw = 0
		self.imu_flag = 0
		self.last_time = rospy.Time.now()


	def writespeed(self, msg):
			global shape_need
			cm_Vx = "{:.2f}".format(float(msg.linear.x))
			cm_Vy = "{:.2f}".format(float(msg.linear.y))
			cm_Wz = "{:.2f}".format(float(msg.angular.z))
			cm_shape = shape_need
			cmd_vel = str(cm_Vx)+","+str(cm_Vy)+","+str(cm_Wz)+","+str(cm_shape)+"\n"
			#print (cmd_vel)
			self.ser.write(cmd_vel.encode())

	def shpneed(self, msg2):
			global shape_need
			shape_need = msg2.data

	def imucallback(self, msg):
		self.imu_msg = msg
		if self.imu_flag == 0:
			(self.roll, self.pitch, self.prev_yaw) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
			self.imu_flag = 1
            
	def encoder_read(self):
		global vx
		global vy
		global w
		global cur_shape
		current_time = rospy.Time.now()
		shape_liz = ["i","o","l"]
		encoder_readings = str(ser.readline())

		if len(encoder_readings)>5:
			encoder_readings = encoder_readings[2:-5].split(",")
			if (len(encoder_readings) == 7):
				encoder_values = encoder_readings[:3]
				print(encoder_readings)
				#cur_shape = shape_liz.index(encoder_readings[3])+1
				
				try:
				    encoder_values = [float(a) for a in encoder_values]
				except:
				    pass
				else:  
				    encoder_values = [float(a) for a in encoder_values]
				    self.vx = encoder_values[0]
				    self.vy = encoder_values[1]
				    self.w = encoder_values[2]
			else:
				print("waiting")
				
			#print(vx,vy,w)
		#b = "0"
		(r, p, yaw) = tf.transformations.euler_from_quaternion([self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w])
		self.w = yaw - self.prev_yaw
		self.prev_yaw = yaw
		dt = (current_time - self.last_time).to_sec()
		delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
		delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
		delta_th = self.w * dt

		self.x += delta_x
		self.y += delta_y
		self.th += delta_th
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"

		# set the position
		odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

		# set the velocity
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.w))

		# publish the message
		self.wheel_odom_pub.publish(odom)
		self.shape_pub.publish(cur_shape)
		self.last_time = current_time



def listener():
    rospy.init_node('smorphi_controller', anonymous=True)
    _object = WASPMini()
    rate = rospy.Rate(20)
    print("Test ok!")
    while not rospy.is_shutdown():
        _object.encoder_read()
        rate.sleep()


if __name__ == '__main__':
    listener()
