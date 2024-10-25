import numpy as np
import time
import argparse
import cv2
from multiprocessing import shared_memory, Array, Lock
import threading

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from teleop.open_television.tv_wrapper import TeleVisionWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK
from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller
from teleop.image_server.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_dir', type=str, default='data', help='path to save data')
    parser.add_argument('--frequency', type=int, default=30.0, help='save data\'s frequency')

    parser.add_argument('--record', action='store_true', help='Save data or not')
    parser.add_argument('--no-record', dest='record', action='store_false', help='Do not save data')
    parser.set_defaults(record=False)

    parser.add_argument('--binocular', action='store_true', help='Use binocular camera')
    parser.add_argument('--monocular', dest='binocular', action='store_false', help='Use monocular camera')
    parser.set_defaults(binocular=True)
    args = parser.parse_args()
    print(f"args:{args}\n")

    # image
    img_shape = (720, 2560, 3)
    img_shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)
    img_array = np.ndarray(img_shape, dtype=np.uint8, buffer=img_shm.buf)
    img_client = ImageClient(img_shape = img_shape, img_shm_name = img_shm.name)
    image_receive_thread = threading.Thread(target=img_client.receive_process, daemon=True)
    image_receive_thread.daemon = True
    image_receive_thread.start()

    # television and arm
    tv_wrapper = TeleVisionWrapper(args.binocular, img_shape, img_shm.name)
    arm_ctrl = G1_29_ArmController()
    arm_ik = G1_29_ArmIK()

    # hand
    left_hand_array = Array('d', 75, lock=True)         # [input]
    right_hand_array = Array('d', 75, lock=True)        # [input]
    dual_hand_data_lock = Lock()
    dual_hand_state_array = Array('d', 14, lock=False)  # [output] current left, right hand state(14) data.
    dual_hand_action_array = Array('d', 14, lock=False) # [output] current left, right hand action(14) data.
    hand_ctrl = Dex3_1_Controller(left_hand_array, right_hand_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array)
    
    if args.record:
        recorder = EpisodeWriter(task_dir=args.task_dir, frequency=args.frequency)
        
    try:
        user_input = input("Please enter the start signal (enter 'r' to start the subsequent program):\n")
        if user_input.lower() == 'r':
            arm_ctrl.speed_gradual_max()
            if args.record:
                press_key_s_count = 0
            
            running = True
            recording = False
            while running:
                start_time = time.time()
                head_rmat, left_wrist, right_wrist, left_hand, right_hand = tv_wrapper.get_data()

                # send hand skeleton data to hand_ctrl.control_process
                left_hand_array[:] = left_hand.flatten()
                right_hand_array[:] = right_hand.flatten()

                # get current state data.
                current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
                current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

                # solve ik using motor data and wrist pose, then use ik results to control arms.
                time_ik_start = time.time()
                sol_q, sol_tauff  = arm_ik.solve_ik(left_wrist, right_wrist, current_lr_arm_q, current_lr_arm_dq)
                time_ik_end = time.time()
                # print(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
                arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

                # record data
                if args.record:
                    with dual_hand_data_lock:
                        left_hand_state = dual_hand_state_array[:7]
                        right_hand_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[-7:]

                    current_image = img_array.copy()
                    left_arm_state  = current_lr_arm_q[:7]
                    right_arm_state = current_lr_arm_q[-7:]
                    left_arm_action = sol_q[:7]
                    right_arm_action = sol_q[-7:]

                    cv2.imshow("record image", current_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        running = False
                    elif key == ord('s'):
                        press_key_s_count += 1
                        if press_key_s_count % 2 == 1:
                            print("==> Start recording...")
                            recording = True
                            recorder.create_episode()
                            print("==> Create episode ok.")
                        else:
                            print("==> End recording...")
                            recording = False
                            recorder.save_episode()
                            print("==> Save episode ok.")

                    if recording:
                        colors = {}
                        depths = {}
                        if args.binocular:
                            colors[f"color_{0}"] = current_image[:, :img_shape[1]//2]
                            colors[f"color_{1}"] = current_image[:, img_shape[1]//2:]
                        else:
                            colors[f"color_{0}"] = current_image
                        states = {
                            "left_arm": {                                                                    
                                "qpos":   left_arm_state.tolist(),    # numpy.array -> list
                                "qvel":   [],                          
                                "torque": [],                        
                            }, 
                            "right_arm": {                                                                    
                                "qpos":   right_arm_state.tolist(),       
                                "qvel":   [],                          
                                "torque": [],                         
                            },                        
                            "left_hand": {                                                                    
                                "qpos":   left_hand_state,           
                                "qvel":   [],                           
                                "torque": [],                          
                            }, 
                            "right_hand": {                                                                    
                                "qpos":   right_hand_state,       
                                "qvel":   [],                           
                                "torque": [],  
                            }, 
                            "body": None, 
                        }
                        actions = {
                            "left_arm": {                                   
                                "qpos":   left_arm_action.tolist(),       
                                "qvel":   [],       
                                "torque": [],      
                            }, 
                            "right_arm": {                                   
                                "qpos":   right_arm_action.tolist(),       
                                "qvel":   [],       
                                "torque": [],       
                            },                         
                            "left_hand": {                                   
                                "qpos":   left_hand_action,       
                                "qvel":   [],       
                                "torque": [],       
                            }, 
                            "right_hand": {                                   
                                "qpos":   right_hand_action,       
                                "qvel":   [],       
                                "torque": [], 
                            }, 
                            "body": None, 
                        }
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / float(args.frequency)) - time_elapsed)
                time.sleep(sleep_time)
                # print(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        img_shm.unlink()
        img_shm.close()
        print("KeyboardInterrupt, exiting program...")
        exit(0)
    finally:
        img_shm.unlink()
        img_shm.close()
        print("Finally, exiting program...")
        exit(0)