"""
Script for check bag files recorded data correctly
"""
#! /usr/bin/env python3

import sys
import cv2
import os
from cv_bridge import CvBridge
import pandas as pd
import rospy
import rosbag

DELTA_T_THRESHOLD = 1 / 15.0

if len(sys.argv) != 4:
    raise Exception("Please input RATE in hz and bag name")

if not sys.argv[1].isnumeric():
    raise Exception("Check rate is in hz")
RATE = float(sys.argv[1])

BAG_NAME = sys.argv[2]
if BAG_NAME[-4:] != ".bag":
    BAG_NAME = BAG_NAME + ".bag"  # Add extension if bag name does not contain it

BAG_PATH = os.path.join("../Fernandez_HAR/bags/", BAG_NAME)
if not os.path.isfile(BAG_PATH):
    raise Exception(f"{os.path.abspath(BAG_PATH)} is not a valid file. Check bag is saved in the correct dir or that bag name is correct")

TO_CHECK = sys.argv[3]
if not TO_CHECK in ["both", "hand", "body"]:
    raise Exception("Which skeleton data do you want to check? input one of the following: 'both', 'hand' or 'body'")


# Init ros stuff
bridge = CvBridge()
rospy.init_node("rosbag_reader", anonymous=True)
rate = rospy.Rate(RATE)

def draw_skel(skel_list, img_):
    """
    Plot circles to the location of the hand joints
    Args:
        skel_list: list of joint locations (image coords, de-normalised)
        img_: img to plot on

    Returns: None
    """
    for j in skel_list:
        x = (round(j[0]), round(j[1]))
        cv2.circle(img=img_, center=x, radius=4, color=(59, 164, 0), thickness=-1)


def check_hand_tracking_data():
    """
    Check hand tracking data was captured correctly
    Returns: None
    """
    img = None
    img_time = None
    hs_left_time = []
    hs_right_time = []
    hs_left_list = []
    hs_right_list = []
    try:
        with rosbag.Bag(BAG_PATH) as bag:
            # Obtain all the images first
            for topic, msg, __ in bag.read_messages(topics=['/mp_rgb_img_comp/compressed', '/hs_left_matched', '/hs_right_matched']):
                if rospy.is_shutdown():
                    break
                if topic == "/mp_rgb_img_comp/compressed":
                    img_time = msg.header.stamp.to_sec()
                    cv2_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    img = cv2_img
                    height_row, width_col = cv2_img.shape[:2]
                elif "left" in topic:
                    if len(msg.poses) > 0:
                        hs_left_time.append(msg.header.stamp.to_sec())
                        curr_joint = []
                        for pose_msg in msg.poses:
                            curr_joint.append([pose_msg.position.x*width_col, pose_msg.position.y*height_row])
                        hs_left_list.append(curr_joint)
                elif "right" in topic:
                    if len(msg.poses) > 0:
                        hs_right_time.append(msg.header.stamp.to_sec())
                        curr_joint = []
                        for pose_msg in msg.poses:
                            curr_joint.append([pose_msg.position.x*width_col, pose_msg.position.y*height_row])
                        hs_right_list.append(curr_joint)
                else:
                    raise Exception(f"topic named '{topic}' does not exist")
                if len(hs_left_time) > 0:
                    if abs(hs_left_time[0] - img_time) <= DELTA_T_THRESHOLD:
                        # Found a matching image
                        draw_skel(hs_left_list[0], img)
                        cv2.imshow('img', img)
                        cv2.waitKey(1)
                        hs_left_time.pop(0)
                        hs_left_list.pop(0)
                if len(hs_right_time) > 0:
                    if abs(hs_right_time[0] - img_time) <= DELTA_T_THRESHOLD:
                        # Found a matching image
                        draw_skel(hs_right_list[0], img)
                        cv2.imshow('img', img)
                        cv2.waitKey(1)
                        hs_right_time.pop(0)
                        hs_right_list.pop(0)
                rate.sleep()

    finally:
        cv2.destroyAllWindows()


def check_body_tracking_data():
    """
    Check body tracking data was captured correctly
    Returns: None
    """
    img = None
    img_time = None
    joint_time = []
    joint_list = []

    try:
        with rosbag.Bag(BAG_PATH) as bag:
            for topic, msg, __ in bag.read_messages(topics=['/nuitrack_rgb_img_comp/compressed', '/body_matched']):
                if rospy.is_shutdown():
                    break
                if topic == '/nuitrack_rgb_img_comp/compressed':
                    img_time = msg.header.stamp.to_sec()
                    cv2_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    img = cv2_img
                elif topic == '/body_matched':
                    if len(msg.poses) > 0:
                        joint_time.append(msg.header.stamp.to_sec())
                        curr_joint = []
                        for pose_msg in msg.poses:
                            curr_joint.append([pose_msg.position.x, pose_msg.position.y])
                        joint_list.append(curr_joint)
                else:
                    raise Exception(f"topic named '{topic}' does not exist")

                try:  # Check for if oldest skeleton data matches current image ts
                    if abs(joint_time[0] - img_time) <= DELTA_T_THRESHOLD:
                        # Found a matching image
                        draw_skel(joint_list[0], img)
                        cv2.imshow('img', img)
                        cv2.waitKey(1)
                        joint_time.pop(0)
                        joint_list.pop(0)
                except Exception as e:
                    pass
                rate.sleep()
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    if TO_CHECK == "hand":
        check_hand_tracking_data()
    elif TO_CHECK == "body":
        check_body_tracking_data()
    elif TO_CHECK == "both":
        check_body_tracking_data()
        check_hand_tracking_data()
    else:
        raise Exception("TO_CHECK input is wrong")


