#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
import sys
import pandas as pd
import os

if len(sys.argv) != 3:
    raise Exception("Please check number of inputs is correct")

FILENAME = f"{sys.argv[1]}/{sys.argv[2]}.csv"
########################################################################################################################
PATH = "/home/louis/Git/HAR_paper_0/Fernandez_HAR/experiment_csvs/"
FILEPATH = os.path.join(PATH, FILENAME)
if not os.path.isfile(FILEPATH):
    raise Exception(f"file: {FILEPATH} not found)")


def rename():
    df = pd.read_csv(FILEPATH, header=None)

    col_1 = ["timestamp"]
    col_2_end = ["pos_x", "pos_y", "pos_z", "orient_x", "orient_y", "orient_z", "orient_w"]
    if "body_skeleton" in FILEPATH:
        joint = ["head_", "neck_", "torso_", "waist_",
                 "left_collar_", "left_shoulder_", "left_elbow_", "left_wrist_", "left_hand_",
                 "right_collar_", "right_shoulder_", "right_elbow_", "right_wrist_", "right_hand_",
                 "left_hip_", "left_knee_", "left_ankle_",
                 "right_hip_", "right_knee_", "right_ankle_"]
    elif ("hs_left" in FILEPATH) or ("hs_right" in FILEPATH):
        joint = ["wrist_",
                 "thumb_cmc_", "thumb_mcp_", "thumb_ip_", "thumb_tpi_",
                 "index_mcp_", "index_pip_", "index_dip_", "index_tip_",
                 "middle_mcp_", "middle_pip_", "middle_dip_", "middle_tip_",
                 "ring_mcp_", "ring_pip_", "ring_dip_", "ring_tip_",
                 "pinky_mcp_", "pinky_pip_", "pinky_dip_", "pinky_tip_"]
    else:
        raise "nuitrack or mediapipe"

    joint_col_named = []

    for i in joint:
        for j in col_2_end:
            joint_col_named.append(i+j)

    df2 = df.set_axis(col_1 + joint_col_named, axis=1)

    new_path = FILEPATH.replace(".csv", "_renamed.csv")
    df2.to_csv(new_path, index=False)


if __name__ == '__main__':
    rename()
