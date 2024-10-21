import numpy as np


T_to_unitree_left_wrist = np.array([[1, 0, 0, 0],
                                    [0, 0, -1, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 0, 1]])

T_to_unitree_right_wrist = np.array([[1, 0, 0, 0],
                                     [0, 0, 1, 0],
                                     [0, -1, 0, 0],
                                     [0, 0, 0, 1]])

T_to_unitree_hand = np.array([[0, 0, 1, 0],
                              [-1,0, 0, 0],
                              [0, -1,0, 0],
                              [0, 0, 0, 1]])

T_robot_openxr = np.array([[0, 0, -1, 0],
                           [-1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 0, 1]])



const_head_vuer_mat = np.array([[1, 0, 0, 0],
                                [0, 1, 0, 1.5],
                                [0, 0, 1, -0.2],
                                [0, 0, 0, 1]])

const_right_wrist_vuer_mat = np.array([[1, 0, 0, 0.5],
                                       [0, 1, 0, 1],
                                       [0, 0, 1, -0.5],
                                       [0, 0, 0, 1]])

const_left_wrist_vuer_mat = np.array([[1, 0, 0, -0.5],
                                      [0, 1, 0, 1],
                                      [0, 0, 1, -0.5],
                                      [0, 0, 0, 1]])

