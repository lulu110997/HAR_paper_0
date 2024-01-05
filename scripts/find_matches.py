"""
Script for finding matching timestamps

- Need to store timestamp as the index for the input
- left and right skeletons have the same timestamps as they were extracted from the same image
"""
import sys
import time
import pandas as pd
import numpy as np
import os
PATH = "../Fernandez_HAR/experiment_csvs/"
FILENAME = "test2_userA/body_skeleton.csv"
FILEPATH = os.path.join(PATH, FILENAME)
########################################################################################################################
df = pd.DataFrame()


def save_and_remove(tuple_matches, tuple_ordered, idx):
    """
    Save matching elements based on index and remove the saved elements from original array
    Args:
        tuple_matches: tuple | where matches are stored
        tuple_ordered: tuple | contains the original array
        idx: list | list of indexes

    Returns: tuple | modified data_dict and data_ordered
    """
    pass
    # if tuple_matches[0] !=
    # matches["body"] = body[idx]
    # matches["hs_right"] = hs_right[idx]
    #
    # # Delete the matches from original arrays
    # body = np.delete(body, idx)
    # hs_right = np.delete(hs_right, idx)

def get_idx_of_matches(data1, data2):
    """
    Find idx where matches between two data containing timestamps occur
    Args:
        data1: 1d np array containing timestamps
        data2: 1d np array containing timestamps

    Returns:

    """
    # Find indexes of matching data
    idx = np.where(np.abs(data1 - data2) < THRESHOLD)[0]
    return consecutive(np.array(idx))[0]

def consecutive(data, stepsize=1):
    """
    https://stackoverflow.com/questions/7352684/how-to-find-the-groups-of-consecutive-elements-in-a-numpy-array
    Args:
        data: np ndarray | array where we look for continuous elements
        stepsize: int | required difference between the previous and current element

    Returns: list | list of numpy arrays containing continuous elements according to stepsize

    """
    return np.split(data, np.where(np.diff(data) != stepsize)[0]+1)

a = np.array([0, 47, 48, 49, 50, 97, 98, 99])
consecutive(a)

# Extract timestamps and find the var with most data recorded
hs_left = pd.read_csv(FILEPATH.replace('body_skeleton', 'hs_left'), header=None).iloc[:, 0].to_numpy(copy=True)
hs_right = pd.read_csv(FILEPATH.replace('body_skeleton', 'hs_right'), header=None).iloc[:, 0].to_numpy(copy=True)
body = pd.read_csv(FILEPATH, header=None).iloc[:, 0].to_numpy(copy=True)

# Ensure all arrays have the same shape
hsl_len = hs_left.shape[0]
hsr_len = hs_right.shape[0]
body_len = body.shape[0]
max_len = np.max([hsl_len, hsr_len, body_len])
if body_len != max_len:
    body = np.pad(body, (0, max_len - body_len), 'constant', constant_values=(0, np.nan))
if hsl_len != max_len:
    hs_left = np.pad(hs_left, (0, max_len - hsl_len), 'constant', constant_values=(0, np.nan))
if hsr_len != max_len:
    hs_right = np.pad(hs_right, (0, max_len - hsr_len), 'constant', constant_values=(0, np.nan))

THRESHOLD = 1/25.0  # Threshold for ts diff to be a match. 25hz since cams run ~30hz + time req for processing
matches = {"body": [], "left": [], "right": []}  # Store matches
unused_data = {"body": [], "left": [], "right": []}  # Store the indexes of unused data

# First find matches between the body and hand skeleton data with most recording
paired_list = zip([hsl_len, hsr_len, body_len], ["hs_left", "hs_right", "body"])
ordered_data = list(zip(*sorted(paired_list, reverse=True)))[1]  # tuple of the data that's sorted in descending order

########################################################################################################################
for i in range(2):

    # Find indexes of matching data
    idx = np.where(np.abs(body - hs_right) < THRESHOLD)[0]
    idx = consecutive(np.array(idx))[0]

    # Save matches
    matches["body"] = body[idx]
    matches["hs_right"] = hs_right[idx]

    # Check the timestamp of the hand data with fewer measurements
    hand_idx = np.where(np.abs(hs_right[idx] - hs_left[idx]) < 0.0001)[0]
    if idx.shape[0] == len(hand_idx):
        # If ts for both hands are equal, it's a match
        matches["hs_right"] = hs_left[idx]
    else:
        # recursive func? Need to align and add 'empty' input
        print("miss")
        pass

    # Check mismatch (which measurement to discard). Didn't take into account how there can be a match between two hands
    # Only thought about no body-hand match :((

    # Delete the matches from original arrays
    print(body)
    body = np.delete(body, idx)
    hs_right = np.delete(hs_right, idx)
    hs_left = np.delete(hs_left, idx)
    print(body)


# Loop until only nan is left in the array with most measurements
while not np.isnan(body).all():
    sys.exit()
    time.sleep(0.001)
    print(body-hs_right)

