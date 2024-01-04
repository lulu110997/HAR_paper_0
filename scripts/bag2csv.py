import numpy as np
import os
import pandas as pd
import rospy
import rosbag

BAG_NAME = "test2_userA.bag"
########################################################################################################################
BAG_PATH = os.path.join("../Fernandez_HAR/2023_12_HAR_bags/", BAG_NAME)
SAVE_PATH = os.path.join("../Fernandez_HAR/experiment_csvs/", BAG_NAME.replace('.bag', ""))
# Create save dir if it doesn't exist
if not os.path.exists(SAVE_PATH):
    os.mkdir(SAVE_PATH)


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
        for topic, msg, __ in bag.read_messages(topics=['/left_hand_skel_data', '/right_hand_skel_data', '/nuitrack_skel_data']):
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
                DEPTH = 1650

            else:
                raise "Unknown topic"

            skel_data["time"].append(msg.header.stamp.to_sec())  # Store timestamp as a float in seconds
            for idx, joints in enumerate(msg.poses):  # Store all joint locations
                joint_lists.extend([joints.position.x/WIDTH, joints.position.y/HEIGHT, joints.position.z/DEPTH,
                                    joints.orientation.x, joints.orientation.y, joints.orientation.z,
                                    joints.orientation.w,])
            if (np.abs(np.array(joint_lists)) > 1.1).any():  # Check all values are normalised. Some values are 1.00001
                np.set_printoptions(precision=4, suppress=True)
                raise Exception(f"smth not normalised. WIDTH={WIDTH}. \n{np.array(joint_lists).reshape(-1,7)}")

            skel_data["data"].append(joint_lists)  # Add to the appropriate df

    # Save dfs
    df = pd.DataFrame(left_hand["data"], index=left_hand["time"])
    df.to_csv(os.path.join(SAVE_PATH, "hs_left.csv"), header=False)
    df = pd.DataFrame(right_hand["data"], index=right_hand["time"])
    df.to_csv(os.path.join(SAVE_PATH, "hs_right.csv"), header=False)
    df = pd.DataFrame(body_skel["data"], index=body_skel["time"])
    df.to_csv(os.path.join(SAVE_PATH, "body_skeleton.csv"), header=False)


hs_data_to_csv()