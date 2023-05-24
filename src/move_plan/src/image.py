# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# 回调函数，接收图像消息并保存为文件
def image_callback(msg):
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = CvBridge().imgmsg_to_cv2(msg, "rgb8")
        # 生成文件名，使用时间戳作为文件名
        filename = rospy.Time.now().to_sec()
        # 保存图像到文件
        path = '/home/tjark/images/'+str(filename) + '.jpg'
        cv2.imwrite(path, cv_image)

        
        rospy.loginfo("Image saved: %s.jpg", str(filename))
    except Exception as e:
        rospy.logerr("Failed to save image: %s", str(e))

def main():
    # 初始化ROS节点
    rospy.init_node("image_saver")

    # 创建图像订阅者
    rospy.Subscriber("/raw_image", Image, image_callback)
 
    # 循环等待回调函数的触发
    rate = rospy.Rate(10)  # 设置循环频率为10Hz
    while not rospy.is_shutdown():
        rospy.spin()  # 处理一次回调函数
      

if __name__ == '__main__':

    main()
