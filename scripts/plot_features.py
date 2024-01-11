import sys
import pandas as pd
import matplotlib.pyplot as plt
import os
import constants as _cs

PATH = "../Fernandez_HAR/experiment_csvs/"
USER = "test3_userA/"
# FILENAME = "body_skeleton.csv"
# FILENAME = "hs_left.csv"
FILENAME = "hs_right.csv"
########################################################################################################################
FILEPATH = os.path.join(PATH, USER, FILENAME)
if not os.path.isfile(FILEPATH):
    print(FILEPATH)

SAVE_IMG_PATH = PATH.replace("experiment_csvs", "results")
SAVE_IMG_PATH = os.path.join(SAVE_IMG_PATH, USER)
if not os.path.exists(SAVE_IMG_PATH):
    os.makedirs(os.path.join(SAVE_IMG_PATH, "hs_left"))
    os.makedirs(os.path.join(SAVE_IMG_PATH, "hs_right"))
    os.makedirs(os.path.join(SAVE_IMG_PATH, "body_skeleton"))

STEP = 7  # Used to iterate through the joint features
df = pd.read_csv(FILEPATH, header=None)
df = df.fillna(0)


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

if "body" in FILENAME:
    joint_names = _cs.BODY_JOINTS
elif "hs" in FILENAME:
    joint_names = _cs.HAND_JOINTS
else:
    raise Exception("what is life")

# Extract features for each joint
for i in range(1, len(df.columns), STEP):
    curr_joint = joint_names[joint_name_counter]
    if "body" in FILENAME and not (curr_joint in _cs.UPPER_JOINTS):
        joint_name_counter += 1
        continue

    # Extract features of each action that will be plotted
    action_features = [[], [], [], []]
    for a_m in actions_measurements:
        for a_idx, a_f in enumerate(action_features):
            a_f.append(df.iloc[a_m[0]:a_m[1]+1, i+a_idx])

    joint_features.append(action_features)
    joint_name_counter += 1

if "left" in FILENAME:
    joint_names = _cs.HAND_JOINTS
    SAVE_IMG_PATH = os.path.join(SAVE_IMG_PATH, "hs_left")
elif "right" in FILENAME:
    joint_names = _cs.HAND_JOINTS
    SAVE_IMG_PATH = os.path.join(SAVE_IMG_PATH, "hs_right")
elif "body" in FILENAME:
    joint_names = _cs.UPPER_JOINTS
    SAVE_IMG_PATH = os.path.join(SAVE_IMG_PATH, "body_skeleton")
else:
    raise Exception("noob")

# Plot features for each joint
for j_idx, a_m in enumerate(joint_features):
    # Create plots for this joint and add titles
    fig, axs = plt.subplots(2, 2)
    fig.suptitle(joint_names[j_idx])

    for f_idx, ax in enumerate(axs.reshape(-1)):
        ax.set_title(subplot_names[f_idx])
        for a_idx, a in enumerate(a_m[f_idx]):
            ax.plot(a, label=action_names[a_idx])
        ax.legend()
    fig.set_size_inches(10, 7.2)
    save_path = os.path.join(SAVE_IMG_PATH, joint_names[j_idx])
    plt.savefig(f"{save_path}.png", dpi=100)
    plt.close()
