import time

import pandas as pd
import numpy as np
import os
PATH = "../Fernandez_HAR/experiment_csvs/"
FILENAME = "test2_userA/body_skeleton.csv"
FILEPATH = os.path.join(PATH, FILENAME)
########################################################################################################################
df = pd.DataFrame()
body = pd.read_csv(FILEPATH, header=None).iloc[:, 0]
hs_left = pd.read_csv(FILEPATH.replace('body_skeleton', 'hs_left'), header=None).iloc[:, 0]
hs_right = pd.read_csv(FILEPATH.replace('body_skeleton', 'hs_right'), header=None).iloc[:, 0]
matches = {"body": [], "left": [], "right": []}

counter = 1
THRESHOLD = 0.04
unused_body_skel = []
df.reset_index()
# print(len(hs_left.diff()))
print(np.max(hs_left.diff().fillna(0).to_numpy()))
