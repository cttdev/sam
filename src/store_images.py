#!/home/cttew/code/sam/ws/src/detection/venv/bin/python

# Stores images taken by a VOXL with mocap location metadata
import numpy as np
import rospy
import cv2
import message_filters
import h5py
from pathlib import Path
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

VOXL_STREAM_TOPIC = "/voxl_cam/voxl_cam/image_raw"
MOCAP_LOCATION_TOPIC = "/mocap_node/pose"

VOXL_CAMERA_RESOLUTION = (640, 480)
BUFFER_SIZE = 100 # frames

bridge = CvBridge()

img_buffer = np.empty((BUFFER_SIZE, VOXL_CAMERA_RESOLUTION[0], VOXL_CAMERA_RESOLUTION[1], 3))
metadata_buffer = np.empty((BUFFER_SIZE, 8)) # x, y, z, qx, qy, qz, qw, timestamp
buffer_position = 0

def dump_buffer():
    # Get the end timestamp of the buffer
    end_time = metadata_buffer[buffer_position - 1][7]

    # Create the file directory
    Path("/data/").mkdir(parents=True, exist_ok=True)

    # Create the file
    file_name = "/data/" + str(end_time) + ".h5"

    # Dump the data
    with h5py.File(file_name, "w") as f:
        f.create_dataset("images", data=img_buffer)
        f.create_dataset("metadata", data=metadata_buffer)

    # Reset the buffer
    buffer_position = 0

def process_data(camera_msg, pose_msg):
    try:
        rospy.loginfo("Received an image!")

        img = bridge.imgmsg_to_cv2(camera_msg, "passthrough")
    except Exception as e:
        rospy.logerr(e)
        return
    
    # Dump the image into the data array
    img_buffer[buffer_position] = img
    metadata_buffer[buffer_position] = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w, pose_msg.header.stamp.to_sec()])
    buffer_position += 1

    if (buffer_position >= BUFFER_SIZE):
        dump_buffer()

def start_node():
    # Initialize the node
    rospy.init_node("capture_node")
    rospy.loginfo("capture_node started!")

    # Create the subscribers
    camera_subscriber = message_filters.Subscriber(VOXL_STREAM_TOPIC, Image)
    rospy.loginfo("Subscribed to %s topic!", VOXL_STREAM_TOPIC)
    
    mocap_subscriber = message_filters.Subscriber(MOCAP_LOCATION_TOPIC, PoseStamped)
    rospy.loginfo("Subscribed to %s topic!", MOCAP_LOCATION_TOPIC)

    ts = message_filters.TimeSynchronizer([camera_subscriber, mocap_subscriber], 50)
    ts.registerCallback(process_data)

    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass