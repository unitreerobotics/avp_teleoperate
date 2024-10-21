from unitree_dds_wrapper.robots.trihand.trihand_pub_cmd import UnitreeTrihand as trihand_pub
from unitree_dds_wrapper.robots.trihand.trihand_sub_state import UnitreeTrihand as trihand_sub

import numpy as np
import time
from multiprocessing import Array
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import threading

unitree_tip_indices = [4, 9, 14]

class Dex3_1_Controller:
    def __init__(self, fps = 100.0):
        self.dex3_pub = trihand_pub()

        kp = np.full(7, 1.5)
        kd = np.full(7, 0.2)
        q = np.full(7,0.0)
        dq = np.full(7,0.0)
        tau = np.full(7,0.0)
        self.dex3_pub.left_hand.kp = kp
        self.dex3_pub.left_hand.kd = kd
        self.dex3_pub.left_hand.q = q 
        self.dex3_pub.left_hand.dq = dq
        self.dex3_pub.left_hand.tau = tau

        self.dex3_pub.right_hand.kp = kp
        self.dex3_pub.right_hand.kd = kd
        self.dex3_pub.right_hand.q = q
        self.dex3_pub.right_hand.dq = dq
        self.dex3_pub.right_hand.tau = tau

        self.dual_hand_state_array = [0.0] * 14
        self.lr_hand_state_lock = threading.Lock()
        # self.dual_hand_state_array = Array('d', 14, lock=True)

        self.sub_state = trihand_sub()
        self.sub_state.wait_for_connection()

        self.subscribe_state_thread = threading.Thread(target=self.subscribe_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        self.hand_retargeting = HandRetargeting(HandType.UNITREE_DEX3)

        self.running = True
        self.fps = fps

        print("UnitreeDex3 Controller init ok.\n")

    def subscribe_state(self):
        while True:
            lq,rq= self.sub_state.sub()
            with self.lr_hand_state_lock:
                self.dual_hand_state_array[:] = np.concatenate((lq, rq))
            # self.dual_hand_state_array[:] = np.concatenate((lq,rq))
            time.sleep(0.002)
            

    def ctrl(self, left_angles, right_angles):
        """set current left, right hand motor state target q"""
        self.dex3_pub.left_hand.q  = left_angles
        self.dex3_pub.right_hand.q  = right_angles
        self.dex3_pub.pub()
        # print("hand ctrl publish ok.")

    def get_current_dual_hand_q(self):
        """return current left, right hand motor state q"""
        with self.lr_hand_state_lock:
            return self.dual_hand_state_array[:].copy()
    
    def control_process(self, left_hand_array, right_hand_array, dual_hand_state_array = None, dual_hand_aciton_array = None):
        left_qpos  = np.full(7, 0)
        right_qpos = np.full(7, 0)
        try:
            while self.running:
                start_time = time.time()
                left_hand_mat  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                right_hand_mat = np.array(right_hand_array[:]).reshape(25, 3).copy()

                is_initial = np.all(left_hand_mat == 0.0)
                if not is_initial:
                    left_qpos  = self.hand_retargeting.left_retargeting.retarget(left_hand_mat[unitree_tip_indices])[[0,1,2,3,4,5,6]]
                    right_qpos = self.hand_retargeting.right_retargeting.retarget(right_hand_mat[unitree_tip_indices])[[0,1,2,3,4,5,6]]

                self.ctrl(left_qpos, right_qpos)

                if dual_hand_state_array and dual_hand_aciton_array:
                    dual_hand_state_array[:] = self.get_current_dual_hand_q()
                    dual_hand_aciton_array[:] = np.concatenate((left_qpos, right_qpos))

                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            print("Dex3_1_Controller has been closed.")