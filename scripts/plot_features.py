import sys

import numpy
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import constants as _cs

PATH = "../Fernandez_HAR/experiment_csvs/"
FILENAME = "test4_userA/body_skeleton.csv"
FILEPATH = os.path.join(PATH, FILENAME)
########################################################################################################################
df = pd.read_csv(FILEPATH, header=None)
df = df.fillna(0)
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
skd.to_csv(FILEPATH.replace(".csv", "_normalised.csv"), header=False, index=False)
# f1 = [1704786281.00035, 1704786295.00036]
# f2 = [1704786299.00036, 1704786327.00036]
# f3 = [1704786345.00035, 1704786373.00035]
# f4 = [1704786378.00034, 1704786391.00034]
# f5 = [1704786398.00035, 1704786427.00036]
# f6 = [1704786434.00035, 1704786439.00035]
# f7 = [1704786447.00037, 1704786457.00036]
# f8 = [1704786461.00035, 1704786471.00034]
# m1 = [1704786481.00035, 1704786493.00034]
# c1 = [1704786498.00035, 1704786512.00035]
f1 = [682, 1102]
f2 = [1222, 2062]
f3 = [2602, 3442]
f4 = [3592, 3982]
f5 = [4192, 5062]
f6 = [5272, 5422]
f7 = [5662, 5962]
f8 = [6082, 6382]
m1 = [6682, 7042]
c1 = [7192, 7612]

# actions_csv_idx = [f1, f2, f3, f4, f5, f6, f7, f8]
# for i in actions_csv_idx:
#     print(i[1] - i[0])
#     print(i)

# axs[0, 0].plot(x, y)
# axs[0, 1].plot(x, y, 'tab:orange')
# axs[0, 1].set_title('Axis [0, 1]')
# axs[1, 0].plot(x, -y, 'tab:green')
# axs[1, 0].set_title('Axis [1, 0]')
# axs[1, 1].plot(x, -y, 'tab:red')
# axs[1, 1].set_title('Axis [1, 1]')
actions_measurements = [f6, f5, m1, c1]
action_names = ["f_least", "f_most", "measure", "cut"]

# Initialise some vars for extracting and plotting features
joint_name_counter = 0
subplot_names = ['x', 'y', 'z', 'angle']
# Contains the features of each action measurement of each joint
# 14 joints (ignore anything below torso --> joint_features
# 4 different action measurements (fasten least, fasten most, measure, cut) --> a_m
# 4 different features (xyz and angle) --> a_f
joint_features = []

# Extract features for each joint
for i in range(1, len(skd.columns), STEP):
    curr_joint = _cs.JOINT_NAMES[joint_name_counter]
    if not (curr_joint in _cs.UPPER_JOINTS):
        joint_name_counter += 1
        continue

    # Extract features of each action that will be plotted
    action_features = [[], [], [], []]
    for a_m in actions_measurements:
        for a_idx, a_f in enumerate(action_features):
            a_f.append(skd.iloc[a_m[0]:a_m[1]+1, i+a_idx])

    joint_features.append(action_features)
    joint_name_counter += 1

# Plot features for each joint
for j_idx, a_m in enumerate(joint_features):
    # Create plots for this joint and add titles
    fig, axs = plt.subplots(2, 2)
    fig.suptitle(_cs.UPPER_JOINTS[j_idx])

    for f_idx, ax in enumerate(axs.reshape(-1)):
        ax.set_title(subplot_names[f_idx])
        for a_idx, a in enumerate(a_m[f_idx]):
            ax.plot(a, label=action_names[a_idx])
        ax.legend()
    fig.set_size_inches(10, 7.2)
    plt.savefig(f"{_cs.UPPER_JOINTS[j_idx]}.png", dpi=100)
    break
# plt.show()

# skd.set_index(0, inplace=True)
# skd.drop(index=skd.index[0], axis=0, inplace=True)
# print(skd.index[1])

