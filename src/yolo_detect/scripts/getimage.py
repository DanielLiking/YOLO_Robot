#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 初始化ROS节点和图像转换器
rospy.init_node('camera_publisher')
bridge = CvBridge()

# 打开摄像头
cap = cv2.VideoCapture(0)

# 创建图像发布者
image_publisher = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)

# 设置循环的频率（以Hz为单位）
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # 读取摄像头图像
    ret, frame = cap.read()

    # 将图像转换为ROS图像消息
    ros_image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')

    # 发布图像消息
    image_publisher.publish(ros_image)

    # 按照指定的频率休眠
    rate.sleep()

# 释放摄像头
cap.release()
