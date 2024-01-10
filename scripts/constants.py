#!/usr/bin/env python3.7
# https://stackoverflow.com/questions/28345780/how-do-i-create-a-python-namespace-argparse-parse-args-value

# # Home directory of the dataset
# DATASET_HOME_DIR = ('/home/louis/Data/Deep learning based robot cognitive architecture '
#                     'for collaborative assembly tasks/har_training_data')
# # DATASET_HOME_DIR = ('/scratch/lffernan/har_training_data')
#
# WEIGHTS = '/home/louis/Github/HAR_replicate_results_Male_J_2023/weights'
#
# # Pickle path should be relative to the paths of the csv files
# PICKLE_SAVE_PATH = DATASET_HOME_DIR + '/pickled_data'
#
# # Contains the user names
# SUBJECT_IDS = ['imu_skeleton_data_user_a_',
#                'imu_skeleton_data_user_b_',
#                'imu_skeleton_data_user_c_',
#                'imu_skeleton_data_user_d_',
#                'imu_skeleton_data_user_e_',
#                'imu_skeleton_data_user_f_',
#                'imu_skeleton_data_user_g_',
#                'imu_skeleton_data_user_h_',
#                'imu_skeleton_data_user_i_',
#                'imu_skeleton_data_user_j_'
#                ]
#
# IMU_SCALERS = ['0_scaler.pickle',
#                '1_scaler.pickle',
#                '2_scaler.pickle',
#                '3_scaler.pickle',
#                '4_scaler.pickle',
#                '5_scaler.pickle',
#                '6_scaler.pickle',
#                '7_scaler.pickle',
#                '8_scaler.pickle',
#                '9_scaler.pickle',
#                '10_scaler.pickle'
#                 ]
#
# # All the csv files
# DATASETS = ["imu_skeleton_data_user_a_take_1.csv",
#             "imu_skeleton_data_user_a_take_2.csv",
#             "imu_skeleton_data_user_b_take_1.csv",
#             "imu_skeleton_data_user_b_take_2.csv",
#             "imu_skeleton_data_user_c_take_1.csv",
#             "imu_skeleton_data_user_c_take_2.csv",
#             "imu_skeleton_data_user_d_take_1.csv",
#             "imu_skeleton_data_user_d_take_2.csv",
#             "imu_skeleton_data_user_e_take_1.csv",
#             "imu_skeleton_data_user_e_take_2.csv",
#             "imu_skeleton_data_user_e_take_3.csv",
#             "imu_skeleton_data_user_e_take_4.csv",
#             "imu_skeleton_data_user_e_take_5.csv",
#             "imu_skeleton_data_user_f_take_1.csv",
#             "imu_skeleton_data_user_f_take_2.csv",
#             "imu_skeleton_data_user_g_take_1.csv",
#             "imu_skeleton_data_user_g_take_2.csv",
#             "imu_skeleton_data_user_h_take_1.csv",
#             "imu_skeleton_data_user_h_take_2.csv",
#             "imu_skeleton_data_user_i_take_1.csv",
#             "imu_skeleton_data_user_i_take_2.csv",
#             "imu_skeleton_data_user_j_take_1.csv",
#             "imu_skeleton_data_user_j_take_2.csv"
#             ]
#
# DATASETS_GROUPED = [
#             ["imu_skeleton_data_user_a_take_1.csv", "imu_skeleton_data_user_a_take_2.csv"],
#             ["imu_skeleton_data_user_b_take_1.csv", "imu_skeleton_data_user_b_take_2.csv"],
#             ["imu_skeleton_data_user_c_take_1.csv", "imu_skeleton_data_user_c_take_2.csv"],
#             ["imu_skeleton_data_user_d_take_1.csv", "imu_skeleton_data_user_d_take_2.csv"],
#             ["imu_skeleton_data_user_e_take_1.csv", "imu_skeleton_data_user_e_take_2.csv",
#              "imu_skeleton_data_user_e_take_3.csv", "imu_skeleton_data_user_e_take_4.csv",
#              "imu_skeleton_data_user_e_take_5.csv"],
#             ["imu_skeleton_data_user_f_take_1.csv", "imu_skeleton_data_user_f_take_2.csv"],
#             ["imu_skeleton_data_user_g_take_1.csv", "imu_skeleton_data_user_g_take_2.csv"],
#             ["imu_skeleton_data_user_h_take_1.csv", "imu_skeleton_data_user_h_take_2.csv"],
#             ["imu_skeleton_data_user_i_take_1.csv", "imu_skeleton_data_user_i_take_2.csv"],
#             ["imu_skeleton_data_user_j_take_1.csv", "imu_skeleton_data_user_j_take_2.csv"]
#             ]
#
# # Action ID of the actions to classify. Null action does not need its own classifier
# ACTIONS_TO_CLASSIFY = [1, 3, 5, 6]
#
# # Names of actions
# ACTION_NAMES = {0: "Null",
#                 1: "Screwdriver_In",
#                 3: "Allen_key_In",
#                 5: "Hammer",
#                 6: "Hand_Screw_In",
#                 8: "Wave",
#                 9: "Forward",
#                 10: "Backward",
#                 11: "Left",
#                 12: "Right",
#                 13: "Stop"
#                 }

# Names of skeletal joints
JOINT_NAMES = ["HEAD",
               "NECK",
               "TORSO",
               "WAIST",
               "LEFT_COLLAR",
               "LEFT_SHOULDER",
               "LEFT_ELBOW",
               "LEFT_WRIST",
               "LEFT_HAND",
               "RIGHT_COLLAR",
               "RIGHT_SHOULDER",
               "RIGHT_ELBOW",
               "RIGHT_WRIST",
               "RIGHT_HAND",
               "LEFT_HIP",
               "LEFT_KNEE",
               "LEFT_ANKLE",
               "RIGHT_HIP",
               "RIGHT_KNEE",
               "RIGHT_ANKLE"
               ]

# Skeletal joints of upper body
UPPER_JOINTS = ["HEAD",
                "NECK",
                "TORSO",
                "WAIST",
                "LEFT_COLLAR",
                "LEFT_SHOULDER",
                "LEFT_ELBOW",
                "LEFT_WRIST",
                "LEFT_HAND",
                "RIGHT_COLLAR",
                "RIGHT_SHOULDER",
                "RIGHT_ELBOW",
                "RIGHT_WRIST",
                "RIGHT_HAND"
                ]