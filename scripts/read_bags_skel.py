import cv2
import numpy as np
import os
from cv_bridge import CvBridge
import pandas as pd
import sys
import rospy
import rosbag
from geometry_msgs.msg import PoseArray, Pose

BAG_NAME = "test1_userA.bag"
BAG_PATH = os.path.join("../Fernandez_HAR/2023_12_HAR_bags/", BAG_NAME)
SAVE_PATH = os.path.join("../Fernandez_HAR/experiment_csvs/", BAG_NAME)

# Create save dir if it doesn't exist
if not os.path.exists(SAVE_PATH):
    os.mkdir(SAVE_PATH)

# Init ros stuff
bridge = CvBridge()
rospy.init_node("rosbag_reader", anonymous=True)
rate = rospy.Rate(30)


def hs_data_to_csv():
    """
    Reads rosbag and converts skeletal data to csv files
    Returns: None
    """
    # Create dicts for storing skeleton positions at each timestep
    left_hand = {'time': [], 'data': []}
    right_hand = {'time': [], 'data': []}
    body_skel = {'time': [], 'data': []}

    with rosbag.Bag(BAG_PATH) as bag:
        for topic, msg, t in bag.read_messages(topics=['/left_hand_skel_data', '/right_hand_skel_data', '/nuitrack_skel_data']):
            if rospy.is_shutdown():
                break
            joint_lists = []  # Store the joint locations for this timestep
            WIDTH = 1; HEIGHT = 1; DEPTH = 1  # Only need to normalise body skeleton (I think)

            if "left" in topic:
                skel_data = left_hand

            elif "right" in topic:
                skel_data = right_hand

            elif "nuitrack" in topic:
                # Need to normalise joint locations in image coordinates as this is not done in nuitrack python SDK
                skel_data = body_skel
                WIDTH = 640
                HEIGHT = 480
                DEPTH = 1500
            else:
                raise "Unknown topic"

            skel_data["time"].append(msg.header.stamp.to_sec())  # Store timestamp as a float in seconds
            for idx, joints in enumerate(msg.poses):  # Store all joint locations
                joint_lists.extend([joints.position.x/WIDTH, joints.position.y/HEIGHT, joints.position.z/DEPTH,
                                    joints.orientation.x, joints.orientation.y, joints.orientation.z,
                                    joints.orientation.w,])
            if (np.abs(np.array(joint_lists)) > 1.1).any():  # Check all values are normalised. Some values are 1.00001
                raise "smth not normalised"

            skel_data["data"].append(joint_lists)  # Add to the appropriate df

    # Save dfs
    df = pd.DataFrame(left_hand["data"], index=left_hand["time"])
    df.to_csv(os.path.join(SAVE_PATH, "hs_right.csv"), header=False)
    df = pd.DataFrame(right_hand["data"], index=right_hand["time"])
    df.to_csv(os.path.join(SAVE_PATH, "hs_left.csv"), header=False)
    df = pd.DataFrame(body_skel["data"], index=body_skel["time"])
    df.to_csv(os.path.join(SAVE_PATH, "body_skeleton.csv"), header=False)


def check_for_matching_ts():
    d = {'mp_rgb_img': [], 'left_hand_skel_data': [], 'right_hand_skel_data': [],
         'nuitrack_rgb_image': [], 'nuitrack_skel_data': []}

    with rosbag.Bag(BAG_PATH) as bag:
        # Obtain all the images first
        for topic, msg, t in bag.read_messages(topics=['/mp_rgb_img', '/left_hand_skel_data', '/right_hand_skel_data',
                                                       '/nuitrack_rgb_image', '/nuitrack_skel_data']):
            if rospy.is_shutdown():
                break
            topic_name = topic[1:]
            d[topic_name].append(t.to_sec())
        df = pd.DataFrame.from_dict(data=d, orient='index').transpose().to_csv("check_ts.csv", index=False)


def change_img(skel_list, img_):
    for j in skel_list:
        x = (round(j[0]), round(j[1]))
        cv2.circle(img=img_, center=x, radius=4, color=(59, 164, 0), thickness=-1)


def check_hand_tracking_data():
    try:
        with rosbag.Bag(BAG_PATH) as bag:
            # Obtain all the images first
            for topic, msg, t in bag.read_messages(topics=['/mp_rgb_img/compressed', '/left_hand_skel_data', '/right_hand_skel_data']):
                if rospy.is_shutdown():
                    break
                if topic == "/mp_rgb_img/compressed":
                    img_time = t.secs + t.nsecs
                    cv2_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    img_list = cv2_img
                    height_row, width_col = cv2_img.shape[:2]
                else:
                    hand_joints = []
                    hand_time = t.secs + t.nsecs
                    try:
                        print(hand_time == img_time)
                        for pose_msg in msg.poses:
                            hand_joints.append([pose_msg.position.x*width_col, pose_msg.position.y*height_row])

                        change_img(hand_joints, img_list)
                        cv2.imshow('img', img_list)
                        cv2.waitKey(1)
                        rate.sleep()
                    except Exception as e:
                        # print(e)
                        pass

    finally:
        cv2.destroyAllWindows()


def check_body_tracking_data():
    img_list = []
    joint_list = []
    try:
        with rosbag.Bag(BAG_PATH) as bag:
            for topic, msg, t in bag.read_messages(topics=['/nuitrack_rgb_image/compressed', '/nuitrack_skel_data']):
                if topic == '/nuitrack_rgb_image/compressed':
                    cv2_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    img_list.append(cv2_img)
                else:
                    joint_now = []
                    for pose_msg in msg.poses:
                        joint_now.append([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
                    joint_list.append(joint_now)
        print("showing images with skeleton data")
        for idx, joint_at_t in enumerate(joint_list):
            if rospy.is_shutdown():
                break
            for j in joint_at_t:
                x = (round(j[0]), round(j[1]))
                cv2.circle(img=img_list[idx], center=x, radius=8, color=(59, 164, 0), thickness=-1)
            cv2.imshow('img', img_list[idx])
            cv2.waitKey(1)
            rate.sleep()
    finally:
        cv2.destroyAllWindows()

hs_data_to_csv()
# check_body_tracking_data()
# check_hand_tracking_data()