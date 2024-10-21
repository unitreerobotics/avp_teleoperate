import numpy as np
import time
import argparse
import cv2
from multiprocessing import Process, shared_memory, Array
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
    parser.add_argument('--record', type=bool, default=True, help='save data or not')
    parser.add_argument('--task_dir', type=str, default='data', help='path to save data')
    parser.add_argument('--frequency', type=int, default=30.0, help='save data\'s frequency')
    args = parser.parse_args()
    print(f"args:{args}\n")

    # image
    img_shape = (480, 640 * 2, 3)
    img_shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)
    img_array = np.ndarray(img_shape, dtype=np.uint8, buffer=img_shm.buf)
    img_client = ImageClient(img_shape = img_shape, img_shm_name = img_shm.name)
    image_receive_thread = threading.Thread(target=img_client.receive_process, daemon=True)
    image_receive_thread.daemon = True
    image_receive_thread.start()

    # television and arm
    tv_wrapper = TeleVisionWrapper(img_shape, img_shm.name)
    arm_ctrl = G1_29_ArmController()
    arm_ik = G1_29_ArmIK()

    # hand
    hand_ctrl = Dex3_1_Controller()
    left_hand_array = Array('d', 75, lock=True)
    right_hand_array = Array('d', 75, lock=True)
    dual_hand_state_array  = Array('d', 14, lock=True) # current left, right hand state data
    dual_hand_aciton_array = Array('d', 14, lock=True) # current left and right hand action data to be controlled
    hand_control_process = Process(target=hand_ctrl.control_process, args=(left_hand_array, right_hand_array, dual_hand_state_array, dual_hand_aciton_array))
    hand_control_process.daemon = True
    hand_control_process.start()
    
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

                # get current arm motor data. solve ik using motor data and wrist pose, then use ik results to control arms.
                time_ik_start = time.time()
                current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
                current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()
                sol_q, sol_tauff  = arm_ik.solve_ik(left_wrist, right_wrist, current_lr_arm_q, current_lr_arm_dq)
                time_ik_end = time.time()
                # print(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
                arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

                # record data
                if args.record:
                    current_image = img_array.copy()
                    left_image =  current_image[:, :640]
                    right_image = current_image[:, 640:]
                    left_arm_state  = current_lr_arm_q[:7]
                    right_arm_state = current_lr_arm_q[-7:]
                    left_hand_state = dual_hand_state_array[:7]
                    right_hand_state = dual_hand_state_array[-7:]
                    left_arm_action = sol_q[:7]
                    right_arm_action = sol_q[-7:]
                    left_hand_action = dual_hand_aciton_array[:7]
                    right_hand_action = dual_hand_aciton_array[-7:]
        
                    cv2.imshow("record image", current_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        running = False
                    elif key == ord('s'):
                        press_key_s_count += 1
                        if press_key_s_count % 2 == 1:
                            print("Start recording...")
                            recording = True
                            recorder.create_episode()
                        else:
                            print("End recording...")
                            recording = False
                            recorder.save_episode()

                    if recording:
                        colors = {}
                        depths = {}
                        colors[f"color_{0}"] = left_image
                        colors[f"color_{1}"] = right_image
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
                                "qpos":   left_hand_state,            # Array returns a list after slicing
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