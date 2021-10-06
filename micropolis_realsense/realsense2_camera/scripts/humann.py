#!/usr/bin/env python3.8

import rospy  # Python library for ROS
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import String
from geometry_msgs.msg import Pose

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)


classNames = []
classFile = '/home/aleyna/catkin_ws/src/pura_vision/scripts/coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
configPath = '/home/aleyna/catkin_ws/src/pura_vision/scripts/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = '/home/aleyna/catkin_ws/src/pura_vision/scripts/frozen_inference_graph.pb'


net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)
    objectInfo = []
    dist_a = []
    min_distance=[]

    dist = 0
    if len(objects) == 0:
        objects = classNames
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            x, y, w, h = box[0], box[1], box[2], box[3]  # NEW
            depth = depth_image[y:y + h, x:x + w].astype(float)
            center_x = int(x + w / 2)
            center_y = int(y + h / 2)
            className = classNames[classId - 1]
            if className in objects:
                objectInfo.append([box, className])
                if (draw):
                    cv2.rectangle(img, box, color=(128, 0, 128), thickness=1)
                    dist = depth_image[center_y, center_x].astype(float)
                    if dist == 0.0:
                        continue
                    dist = dist / 1000
                    dist_a.append(dist)
                    min_distance=min(dist_a)  # min distance
                    cv2.putText(color_image, "{0:.2f}".format(dist), (box[0], box[1] - 20),
                                cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

                    cv2.putText(img, className.upper(), (x + 10, y + 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(img, str(round(confidence * 100, 2)), (x + 200, y + 30), cv2.FONT_HERSHEY_COMPLEX, 1,
                                (0, 255, 0), 2)

    return img, objectInfo,min_distance



def publish_message(color_image,human,min_dis):
        a = 0
        pub = rospy.Publisher('video_frames', Image, queue_size=10)
        pub1 = rospy.Publisher('human_count', String, queue_size=10)
        pub2 = rospy.Publisher('human_distance', String, queue_size=10)
        rate = rospy.Rate(10)  # 10hz
        br = CvBridge()


        while not rospy.is_shutdown():


            rospy.loginfo('publishing video frame')
            pub.publish(br.cv2_to_imgmsg(color_image))
            str1 = " {}".format(human)
            str2 = " {}".format(min_dis)
            pub1.publish(str1)
            pub2.publish(str2)
            rate.sleep()
            return a


if __name__ == '__main__':
    try:
        rospy.init_node('video_pub_py', anonymous=True)

        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            result, objectInfo, min_distance = getObjects(color_image, 0.6, 0.2, True, objects=['person'])
            cv2.putText(color_image, str(len(objectInfo)), (20, 30), cv2.FONT_HERSHEY_COMPLEX, 1,
                        (0, 255, 0), 2)
            publish_message(color_image,str(len(objectInfo)), min_distance)
            cv2.waitKey(1)

    except rospy.ROSInterruptException:
        pass
    finally:
        pipeline.stop()