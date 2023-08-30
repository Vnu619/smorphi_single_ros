#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import serial

rospy.init_node('odom_publisher')


def writespeed(msg):
    global shape_need
    cm_Vx = "{:.2f}".format(float(msg.linear.x))
    cm_Vy = "{:.2f}".format(float(msg.linear.y))
    cm_Wz = "{:.2f}".format(float(msg.angular.z))
    cm_shape = shape_need
    cmd_vel = str(cm_Vx)+","+str(cm_Vy)+","+str(cm_Wz)+","+str(cm_shape)+"\n"
    #print (cmd_vel)
    ser.write(cmd_vel.encode())



def shpneed(msg2):
    global shape_need
    shape_need = msg2.data

#rospy.Subscriber("platform_speeds", String, encoder_read)
sub = rospy.Subscriber("/cmd_vel", Twist, writespeed)
sub_shape_need = rospy.Subscriber("/shape_need", Int32, shpneed)

odom_pub = rospy.Publisher("wheel_odom", Odometry, queue_size=50)
shape_pub = rospy.Publisher("current_shape", Int32, queue_size=50)

odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
w = 0.0

shape_need = 0
cur_shape = 0

ser = serial.Serial('/dev/ttyUSB2',115200, timeout=0.005)



def encoder_read():
    global vx
    global vy
    global w
    global cur_shape
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
                vx = encoder_values[0]
                vy = encoder_values[1]
                w = encoder_values[2]
        else:
            print("waiting")
            
        #print(vx,vy,w)
b = "0"

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(100)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
        
    encoder_read()
    #a = ser.readline()
    #if len(a)>0:
        #b = a
    #print (b)
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = w * dt

    x += delta_x
    y += delta_y
    th += delta_th
    #print("x: ", x,"y: ", y,"th: ", th)
    #print("x", x)
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    #odom_broadcaster.sendTransform(
    #    (x, y, 0.),
    #    odom_quat,
    #    current_time,
    #    "base_link",
    #    "odom"
    #)

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, w))

    # publish the message
    odom_pub.publish(odom)
    shape_pub.publish(cur_shape)
    
    last_time = current_time
    r.sleep()
