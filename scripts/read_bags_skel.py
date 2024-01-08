#! /usr/bin/env python3
import cv2
import os
from cv_bridge import CvBridge
import pandas as pd
import rospy
import rosbag

BAG_NAME = "test3_userA.bag"
BAG_PATH = os.path.join("../Fernandez_HAR/bags/", BAG_NAME)

# Init ros stuff
bridge = CvBridge()
rospy.init_node("rosbag_reader", anonymous=True)
rate = rospy.Rate(240)

def show_img(msg):
    """
    msg: img msg to show
    use cv2 to show img
    Returns: None
    """
    cv2_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
    cv2.imshow('win', cv2_img)
    cv2.waitKey(1)


def check_pics():
    """
    Check imgs based on nuitrack ts to ensure start and end time of action is correct
    Returns: None
    """
    try:
        with rosbag.Bag(BAG_PATH) as bag:
            for topic, msg, t in bag.read_messages(topics=['/nuitrack_rgb_img_comp/compressed']):
                if rospy.is_shutdown():
                    break
                if 1704331925 < msg.header.stamp.secs < 1704331944:
                    show_img(msg)
                    rate.sleep()
                elif 1704331950 < msg.header.stamp.secs < 1704331956:
                    show_img(msg)
                    rate.sleep()
                elif 1704331962 < msg.header.stamp.secs < 1704331984:
                    show_img(msg)
                    rate.sleep()
                elif 1704332001 < msg.header.stamp.secs < 1704332007:
                    show_img(msg)
                    rate.sleep()
                elif 1704332017 < msg.header.stamp.secs < 1704332027:
                    show_img(msg)
                    rate.sleep()
    finally:
        cv2.destroyAllWindows()




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


def draw_hand(skel_list, img_):
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
    try:
        with rosbag.Bag(BAG_PATH) as bag:
            # Obtain all the images first
            for topic, msg, t in bag.read_messages(topics=['/mp_rgb_img_comp/compressed', '/hs_left_matched', '/hs_right_matched']):
                if rospy.is_shutdown():
                    break
                if topic == "/mp_rgb_img_comp/compressed":
                    img_time = msg.header.stamp.to_sec()
                    cv2_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    img_list = cv2_img
                    height_row, width_col = cv2_img.shape[:2]
                elif "hs_" in topic:
                    hand_joints = []
                    hand_time = msg.header.stamp.to_sec()
                else:
                    raise Exception(f"topic named '{topic}' does not exist")
                try:
                    if (1/15.0) < abs(hand_time - img_time) < (1/45.0):
                        continue
                    for pose_msg in msg.poses:
                        hand_joints.append([pose_msg.position.x*width_col, pose_msg.position.y*height_row])

                    draw_hand(hand_joints, img_list)
                    cv2.imshow('img', img_list)
                    cv2.waitKey(1)
                    rate.sleep()

                except Exception as e:
                    # print(e)
                    pass

    finally:
        cv2.destroyAllWindows()


def check_body_tracking_data():
    """
    Check body tracking data was captured correctly
    Returns: None
    """
    img_list = []
    joint_list = []
    try:
        with rosbag.Bag(BAG_PATH) as bag:
            for topic, msg, t in bag.read_messages(topics=['/nuitrack_rgb_img_comp/compressed', '/body_matched']):
                if topic == '/nuitrack_rgb_img_comp/compressed':
                    cv2_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    img_list.append(cv2_img)
                elif topic == '/body_matched':
                    joint_now = []
                    for pose_msg in msg.poses:
                        joint_now.append([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
                    joint_list.append(joint_now)
                else:
                    raise Exception(f"topic named '{topic}' does not exist")
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

# check_body_tracking_data()
check_hand_tracking_data()
# check_pics()