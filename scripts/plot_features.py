import numpy
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

PATH = "../Fernandez_HAR/experiment_csvs/"
FILENAME = "test2_userA/body_skeleton.csv"
FILEPATH = os.path.join(PATH, FILENAME)
########################################################################################################################
df = pd.read_csv(FILEPATH, header=None)
if not os.path.isfile(FILEPATH):
    print(FILEPATH)
STEP = 7

def _normalise_skeleton_joints(df):
    """
    Normalises joints in the skeleton dataframe.
    For each frame, the joint locations are normalised relative to the:
    - length between MIDDLE_MCP and WRIST for hand joints
    - length between TORSO and WAIST for body joints

    Args:
        df: dataframe | contains the joint (col) pose/orientation per frame (rows)

    Returns:
        skeleton_df: dataframe | contains normalised joint locations
    """

    skeleton_df = df.copy(deep=True)

    if len(skeleton_df.columns) == 148:  # Hand skeleton
        JOINT_1 = skeleton_df.iloc[:, 63:66].to_numpy(copy=True)  # MIDDLE_MCP
        JOINT_2 = skeleton_df.iloc[:, 1:4].to_numpy(copy=True)  # WRIST
    elif len(skeleton_df.columns) == 141:  # Body skeleton
        JOINT_1 = skeleton_df.iloc[:, 15:18].to_numpy(copy=True)  # TORSO
        JOINT_2 = skeleton_df.iloc[:, 22:25].to_numpy(copy=True)  # WAIST
    else:
        raise "No. columns doesn't add up"

    for i in range(1, len(skeleton_df.columns), STEP):
        # Obtain the current joint's xyz position
        current_joint_xyz = skeleton_df.iloc[:, i:i+3].to_numpy(copy=True)
        # if i < 2:
        #     try:
        #         t_lt = skeleton_df.loc[9059].iloc[i:i+3]
        #         print("success")
        #         print(skeleton_df.loc[9059].iloc[8:11])
        #         print(skeleton_df.loc[9059].iloc[15:18])
        #         print(t_lt)
        #         print('success')
        #     except Exception as e:
        #         pass
        # Normalise joint locations
        a = current_joint_xyz - JOINT_1
        b = JOINT_1 - JOINT_2
        skeleton_df.iloc[:, i:i + 3] = np.divide(a, b, out=np.zeros(a.shape), where=b != 0)
        # skeleton_df.iloc[:, i:i + 3] = a/b

    return skeleton_df


skd = _normalise_skeleton_joints(df)
# skd.to_csv(FILEPATH.replace(".csv", "_normalised.csv"), header=False, index=False)

