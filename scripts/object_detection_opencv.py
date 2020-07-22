#!/usr/bin/env python

import sys
import time

# numpy and scipy
import numpy as np
import cv2

# Ros
import rospy
from sensor_msgs.msg import CompressedImage
import rospkg
from std_msgs.msg import Int16MultiArray

def get_output_layers(net):

    layer_names = net.getLayerNames()

    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    return output_layers


def draw_prediction(img, label, color, confidence, x, y, x_plus_w, y_plus_h):

    cv2.rectangle(img, (int(x),int(y)), (int(x_plus_w),int(y_plus_h)), (int(color[0]), int(color[1]), int(color[2])), 2)

    cv2.putText(img, label, (int(x-10),int(y-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


class ObjectDetectorOpenCV:

    def __init__(self):
        # Read input parameters
        self.input_image_compressed = rospy.get_param('~input_image_compressed', "usb_cam/image_raw/compressed")
        self.output_image_compressed = rospy.get_param('~output_image', "face_image/compressed")
        self.model_path_prefix = rospy.get_param('~model_path_prefix', "models/yolov3")
        self.tracked_object = rospy.get_param('~tracked_object', "apple")

        # print input parameters
        rospy.loginfo("input_image_compressed: " + self.input_image_compressed)
        rospy.loginfo("output_image_compressed: " + self.output_image_compressed)

        self.current_image = CompressedImage()

        rospy.loginfo("Loading YOLO model")

        self.pub_image = rospy.Publisher(self.output_image_compressed, CompressedImage, queue_size=1)
        self.pub_box = rospy.Publisher("bounding_box", Int16MultiArray, queue_size=1)

        self.subscriber = rospy.Subscriber(self.input_image_compressed,  CompressedImage, self.callback, queue_size=1)

        rospy.loginfo("detection started")

        self.classes = None

        with open(self.model_path_prefix+'.txt', 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.COLORS = np.random.uniform(0, 255, size=(len(self.classes), 3))

        self.net = cv2.dnn.readNet(self.model_path_prefix+'.weights', self.model_path_prefix+'.cfg')


        while not rospy.is_shutdown():
            self.process_current_image()
            time.sleep(1)

        #rospy.spin()


    def process_current_image(self):
        # No image data received
        if len(self.current_image.data) == 0:
            return

        # skip is no subscribers request for detections
        if self.pub_box.get_num_connections() == 0 and self.pub_image.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscriber to connect")
            return

        np_arr = np.fromstring(self.current_image.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if self.pub_box.get_num_connections() > 0:
            Width = image.shape[1]
            Height = image.shape[0]
            scale = 0.00392

            blob = cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)

            self.net.setInput(blob)

            outs = self.net.forward(get_output_layers(self.net))

            class_ids = []
            confidences = []
            boxes = []
            conf_threshold = 0.1
            nms_threshold = 0.4


            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > conf_threshold:
                        center_x = int(detection[0] * Width)
                        center_y = int(detection[1] * Height)
                        w = int(detection[2] * Width)
                        h = int(detection[3] * Height)
                        x = center_x - w / 2
                        y = center_y - h / 2
                        class_ids.append(class_id)
                        confidences.append(float(confidence))
                        boxes.append([x, y, w, h])


            indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

            for i in indices:
                i = i[0]

                class_id = class_ids[i]
                label = str(self.classes[class_id])

                if label == self.tracked_object:
                    box = boxes[i]
                    x = box[0]
                    y = box[1]
                    w = box[2]
                    h = box[3]
                    box_msg = Int16MultiArray()
                    box_msg.data = [class_id, x, y, x+w, y+h]
                    self.pub_box.publish(box_msg)
                    color = self.COLORS[class_id]
                    draw_prediction(image, label, color, confidences[i], round(x), round(y), round(x+w), round(y+h))

        # skip if no subscribers are registered
        if self.pub_image.get_num_connections() == 0:
            return

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()

        # Publish image with face detections
        self.pub_image.publish(msg)

    def callback(self, ros_data):
        self.current_image = ros_data


def main(args):
    rospy.init_node('ObjectDetectorOpenCV')

    ObjectDetectorOpenCV()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
