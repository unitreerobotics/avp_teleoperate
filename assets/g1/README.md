# Unitree G1 Description (URDF & MJCF)

## Overview

This package includes a streamlined robot description (URDF & MJCF) for the [Unitree G1](https://www.unitree.com/g1/), developed by [Unitree Robotics](https://www.unitree.com/).

<p align="center"><img src="g1.png" width="500"/></p>

Unitree G1 have 43 DOFs:

```text
root [⚓] => /pelvis/
    left_hip_pitch_joint [⚙+Y] => /left_hip_pitch_link/
        left_hip_roll_joint [⚙+X] => /left_hip_roll_link/
            left_hip_yaw_joint [⚙+Z] => /left_hip_yaw_link/
                left_knee_joint [⚙+Y] => /left_knee_link/
                    left_ankle_pitch_joint [⚙+Y] => /left_ankle_pitch_link/
                        left_ankle_roll_joint [⚙+X] => /left_ankle_roll_link/
    right_hip_pitch_joint [⚙+Y] => /right_hip_pitch_link/
        right_hip_roll_joint [⚙+X] => /right_hip_roll_link/
            right_hip_yaw_joint [⚙+Z] => /right_hip_yaw_link/
                right_knee_joint [⚙+Y] => /right_knee_link/
                    right_ankle_pitch_joint [⚙+Y] => /right_ankle_pitch_link/
                        right_ankle_roll_joint [⚙+X] => /right_ankle_roll_link/
    waist_yaw_joint [⚙+Z] => /waist_yaw_link/
        waist_roll_joint [⚙+X] => /waist_roll_link/
            waist_pitch_joint [⚙+Y] => /torso_link/
                logo_joint [⚓] => /logo_link/
                head_joint [⚓] => /head_link/
                left_shoulder_pitch_joint [⚙+Y] => /left_shoulder_pitch_link/
                    left_shoulder_roll_joint [⚙+X] => /left_shoulder_roll_link/
                        left_shoulder_yaw_joint [⚙+Z] => /left_shoulder_yaw_link/
                            left_elbow_joint [⚙+Y] => /left_elbow_link/
                                left_wrist_roll_joint [⚙+X] => /left_wrist_roll_link/
                                    left_wrist_pitch_joint [⚙+Y] => /left_wrist_pitch_link/
                                        left_wrist_yaw_joint [⚙+Z] => /left_wrist_yaw_link/
                                            left_hand_palm_joint [⚓] => /left_hand_palm_link/
                                                left_hand_zero_joint [⚙+Y] => /left_hand_zero_link/
                                                    left_hand_one_joint [⚙+Z] => /left_hand_one_link/
                                                        left_hand_two_joint [⚙+Z] => /left_hand_two_link/
                                                left_hand_three_joint [⚙+Z] => /left_hand_three_link/
                                                    left_hand_four_joint [⚙+Z] => /left_hand_four_link/
                                                left_hand_five_joint [⚙+Z] => /left_hand_five_link/
                                                    left_hand_six_joint [⚙+Z] => /left_hand_six_link/
                right_shoulder_pitch_joint [⚙+Y] => /right_shoulder_pitch_link/
                    right_shoulder_roll_joint [⚙+X] => /right_shoulder_roll_link/
                        right_shoulder_yaw_joint [⚙+Z] => /right_shoulder_yaw_link/
                            right_elbow_joint [⚙+Y] => /right_elbow_link/
                                right_wrist_roll_joint [⚙+X] => /right_wrist_roll_link/
                                    right_wrist_pitch_joint [⚙+Y] => /right_wrist_pitch_link/
                                        right_wrist_yaw_joint [⚙+Z] => /right_wrist_yaw_link/
                                            right_hand_palm_joint [⚓] => /right_hand_palm_link/
                                                right_hand_zero_joint [⚙+Y] => /right_hand_zero_link/
                                                    right_hand_one_joint [⚙+Z] => /right_hand_one_link/
                                                        right_hand_two_joint [⚙+Z] => /right_hand_two_link/
                                                right_hand_three_joint [⚙+Z] => /right_hand_three_link/
                                                    right_hand_four_joint [⚙+Z] => /right_hand_four_link/
                                                right_hand_five_joint [⚙+Z] => /right_hand_five_link/
                                                    right_hand_six_joint [⚙+Z] => /right_hand_six_link/
```

## Visulization with [MuJoCo](https://github.com/google-deepmind/mujoco)

1. Open MuJoCo Viewer

   ```bash
   pip install mujoco

   python -m mujoco.viewer
   ```

2. Drag and drop the MJCF/URDF model file (`g1_body29_hand14.xml`/`g1_body29_hand14.urdf`) to the MuJoCo Viewer.
