import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2
import cv2
from std_msgs.msg import String


if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2



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
    center_x=[]
    center_y=[]
    dist = 0
    if len(objects) == 0:
        objects = classNames
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            x, y, w, h = box[0], box[1], box[2], box[3]  # NEW
            center_x = int(x + w / 2)
            center_y = int(y + h / 2)
            className = classNames[classId - 1]
            if className in objects:
                objectInfo.append([box, className])
                if (draw):
                    cv2.rectangle(img, box, color=(128, 0, 128), thickness=1)
                    cv2.putText(img, className.upper(), (x + 10, y + 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(img, str(round(confidence * 100, 2)), (x + 200, y + 30), cv2.FONT_HERSHEY_COMPLEX, 1,
                                (0, 255, 0), 2)

    return img, objectInfo,center_x,center_y


class ImageListener:
    def __init__(self, image_topic,depth_image_topic):
        self.bridge = CvBridge()
        self.sub_info = rospy.Subscriber(image_topic, msg_Image, self.imageInfoCallback)
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        #confidence_topic = depth_image_topic.replace('depth', 'confidence')
        #self.sub_conf = rospy.Subscriber(confidence_topic, msg_Image, self.confidenceCallback)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

    def imageInfoCallback(self, cameraInfo):
        try:
            image = self.bridge.imgmsg_to_cv2(cameraInfo, cameraInfo.encoding)
            result, objectInfo, self.center_x,self.center_y = getObjects(image, 0.6, 0.2, True, objects=['person'])
            cv2.putText(image, str(len(objectInfo)), (20, 30), cv2.FONT_HERSHEY_COMPLEX, 1,
                        (0, 255, 0), 2)
            a = 0
            pub11 = rospy.Publisher('video_frames', msg_Image, queue_size=10)
            pub12 = rospy.Publisher('human_count', String, queue_size=10)
            rate = rospy.Rate(10)  # 10hz

            while not rospy.is_shutdown():
                br=CvBridge()
                rospy.loginfo('publishing video frame')
                pub11.publish(br.cv2_to_imgmsg(result))
                str1 = " {}".format( str(len(objectInfo)))
                pub12.publish(str1)
                rate.sleep()
                cv2.waitKey(1)
                return a

        except CvBridgeError as e:
            print(e)

    def imageDepthCallback(self, data2):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data2, data2.encoding)
                min_distance=cv_image[self.center_y, self.center_x]
                min_distance=min_distance/1000
                line = '\rDepth at pixel(%3d, %3d): %f(cm).' % (self.center_x, self.center_y,min_distance)
                pub13 = rospy.Publisher('human_distance', String, queue_size=10)
                str2 = " {}".format(min_distance)
                pub13.publish(str2)
                sys.stdout.write(line)
                sys.stdout.flush()

            except CvBridgeError as e:
                print(e)
                return
            except ValueError as e:
                return


if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    image_topic = '/camera/color/image_raw'
    depth_image_topic = '/camera/depth/image_rect_raw'
    listener = ImageListener(image_topic,depth_image_topic)
    rospy.spin()
