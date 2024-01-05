"""
Script for finding matching timestamps

find matches between two recordings with most data the save those timestamps in a variable
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



# First compare the two data with most recordings then compare those timestamps with data with the least recording
paired_list = zip([hsl_len, hsr_len, body_len], ["hs_left", "hs_right", "body"])
ordered_data = list(zip(*sorted(paired_list, reverse=True)))[1]  # tuple of the data that's sorted in descending order

# Find indexes of matching data
# get_idx_of_matches(ordered_data[0], ordered_data[1])
idx = np.where(np.abs(body - hs_right) < 1/25.0)[0]
idx = consecutive(np.array(idx))[0]

# Save matches
matches["body"] = body[idx]
matches["hs_right"] = hs_right[idx]

# Delete the matches from original arrays
body = np.delete(body, idx)
hs_right = np.delete(hs_right, idx)

# Do the above for the data with min recording


# Loop until only nan is left in the array
while not np.isnan(body).all():
    sys.exit()
    time.sleep(0.001)
    print(body-hs_right)

