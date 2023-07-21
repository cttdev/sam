#!/home/cttew/code/sam/ws/src/detection/venv/bin/python
import numpy as np
import rospy
import cv2
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

CAMERA_IMAGE_TOPIC = "/camera/color/image_raw"
DEPTH_IMAGE_TOPIC = "/camera/aligned_depth_to_color/image_raw"

FPS = 30

model = YOLO("yolov8n-seg.pt")
bridge = CvBridge()

def process_image(color_msg):
    try:
        rospy.loginfo("Received an image!")

        color_img = bridge.imgmsg_to_cv2(color_msg, "passthrough")
        # depth_img = bridge.imgmsg_to_cv2(depth_msg, "passthrough")
    except Exception as e:
        rospy.logerr(e)

    # cv2.imshow("Color", color_img)
    # # cv2.imshow("Depth", depth_img)
    # cv2.waitKey(0)

    # Run the model
    results = model(color_img, show=True)
    # best_result = results[0]

    # # Get the mask
    # if(best_result is not None):
    #     # Convert mask to single channel image
    #     mask_raw = best_result.masks[0].numpy()

        

    # print(results)

def start_node():
    # Initialize the node
    rospy.init_node("detection_node")
    rospy.loginfo("detection_node started!")

    # Create the subscribers
    color_subscriber = message_filters.Subscriber(CAMERA_IMAGE_TOPIC, Image)
    rospy.loginfo("Subscribed to %s topic!", CAMERA_IMAGE_TOPIC)
    
    depth_subscriber = message_filters.Subscriber(DEPTH_IMAGE_TOPIC, Image)
    rospy.loginfo("Subscribed to %s topic!", DEPTH_IMAGE_TOPIC)

    ts = message_filters.ApproximateTimeSynchronizer([color_subscriber], 10, 1.0 / FPS)
    ts.registerCallback(process_image)

    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass