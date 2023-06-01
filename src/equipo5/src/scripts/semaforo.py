#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Twist

color_light = 0

def callback(msg):
    global cv_image
    global color_light

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    red_mask = cv2.inRange(cv_image, (0, 0, 100), (50, 50, 255))
    yellow_mask = cv2.inRange(cv_image, (20, 100, 100), (30, 255, 255))
    green_mask = cv2.inRange(cv_image, (0, 100, 0), (50, 255, 50))

    red_result = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)
    yellow_result = cv2.bitwise_and(cv_image, cv_image, mask=yellow_mask)
    green_result = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)

    red_gray = cv2.cvtColor(red_result, cv2.COLOR_BGR2GRAY)
    yellow_gray = cv2.cvtColor(yellow_result, cv2.COLOR_BGR2GRAY)
    green_gray = cv2.cvtColor(green_result, cv2.COLOR_BGR2GRAY)

    red_gray = cv2.GaussianBlur(red_gray, (5, 5), 0)
    yellow_gray = cv2.GaussianBlur(yellow_gray, (5, 5), 0)
    green_gray = cv2.GaussianBlur(green_gray, (5, 5), 0)

    red_circle = cv2.HoughCircles(red_gray, cv2.HOUGH_GRADIENT, 3 , 20, param1=100, param2=30, minRadius=10, maxRadius=0)
    yellow_circle = cv2.HoughCircles(yellow_gray, cv2.HOUGH_GRADIENT, 3 , 20, param1=100, param2=30, minRadius=10, maxRadius=0)
    green_circle = cv2.HoughCircles(green_gray, cv2.HOUGH_GRADIENT, 3 , 20, param1=100, param2=30, minRadius=10, maxRadius=0)

    if red_circle is not None:
        color_light = 0
        red_circle = np.round(red_circle[0, :]).astype("int")
        for (x, y, r) in red_circle:
            cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)

    if yellow_circle is not None:
        color_light = 1
        yellow_circle = np.round(yellow_circle[0, :]).astype("int")
        for (x, y, r) in yellow_circle:
            cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)

    if green_circle is not None:
        color_light = 2
        green_circle = np.round(green_circle[0, :]).astype("int")
        for (x, y, r) in green_circle:
            cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)

def stop():
    print("stop")



if _name_ == '_main_':
    print("Running")
    rospy.init_node('traffic_light')
    rospy.Subscriber('/video_source/raw', Image, callback)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    while not rospy.is_shutdown():


        twist = Twist()
        if color_light == 0:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif color_light == 1:  
            twist.linear.x = 0.5
            twist.angular.z = 0.0
        elif color_light == 2:  
            twist.linear.x = 2.0
            twist.angular.z = 0.0
        else:  # Unknown
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        pub_cmd_vel.publish(twist)

        print(color_light)
        print("\n")

        rate.sleep()
