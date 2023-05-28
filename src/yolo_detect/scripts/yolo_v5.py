#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes


class Yolo_Dect:
    def __init__(self):
        # load parameters
        yolov5_path = rospy.get_param('/yolov5_path', '')
        weight_path = rospy.get_param('~weight_path', '')
        conf = rospy.get_param('~conf', '0.5')

        # load local repository(YoloV5:v6.0)
        self.model = torch.hub.load(yolov5_path, 'custom',
                                    path=weight_path, source='local')

        # which device will be used
        if rospy.get_param('/use_cpu', 'false'):
            self.model.cpu()
        else:
            self.model.cuda()

        self.model.conf = conf

        # Load class color
        self.classes_colors = {}

        self.position_pub = rospy.Publisher(
            '/yolov5/BoundingBoxes', BoundingBoxes, queue_size=1)

        self.cap = cv2.VideoCapture(0)  # Open the camera

    def detect_objects(self):
        while True:
            ret, frame = self.cap.read()  # Read frames from the camera

            results = self.model(frame)  # Perform object detection

            # Build BoundingBoxes message
            bounding_boxes = BoundingBoxes()
            bounding_boxes.header = Header()
            bounding_boxes.header.stamp = rospy.Time.now()

            boxs = results.pandas().xyxy[0].values
            self.draw_boxes(frame, boxs)

            for box in boxs:
                bounding_box = BoundingBox()
                bounding_box.probability = box[4]
                bounding_box.xmin = box[0]
                bounding_box.ymin = box[1]
                bounding_box.xmax = box[2]
                bounding_box.ymax = box[3]
                bounding_box.num = 1
                bounding_box.Class = box[-1]
                bounding_boxes.bounding_boxes.append(bounding_box)

                #计算边界框中心点与相机光轴的夹角
                angle = self.calculate_angle(frame,bounding_box)
                bounding_box.angel = angle
            self.position_pub.publish(bounding_boxes)  # Publish BoundingBoxes message

            cv2.imshow('YOLOv5', frame)  # Display the image
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
                break

    def draw_boxes(self, img, boxes):
        for box in boxes:
            if box[-1] in self.classes_colors.keys():
                color = self.classes_colors[box[-1]]
            else:
                color = np.random.randint(0, 183, 3)
                self.classes_colors[box[-1]] = color

            cv2.rectangle(img, (int(box[0]), int(box[1])),
                          (int(box[2]), int(box[3])), (int(color[0]), int(color[1]), int(color[2])), 2)

            if box[1] < 20:
                text_pos_y = box[1] + 30
            else:
                text_pos_y = box[1] - 10

            cv2.putText(img, box[-1],
                        (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    def calculat_angle(self, frame, bounding_box):
        height, width, _ = frame.shape

        #计算边界框中心点坐标
        center_x = (bounding_box.xmin + bounding_box.xmax) / 2
        center_y = (bounding_box.ymin + bounding_box.ymax) / 2

        #计算边界框中心点与图像中心点的偏移量
        dx = center_x - width / 2
        dy = center_y - height / 2


        #计算边界框中心点与相机光轴的夹角
        angle = np.arctan2(dy, dx)
        return angle
    def release_camera(self):
        self.cap.release()  # Release the camera
        cv2.destroyAllWindows()  # Close the window


    def
def main():
    rospy.init_node('yolov5_ros', anonymous=True)
    yolo_dect = Yolo_Dect()
    try:
        yolo_dect.detect_objects()
    finally:
        yolo_dect.release_camera()


if __name__ == "__main__":
    main()
