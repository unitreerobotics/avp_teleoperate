import numpy as np

from TeleVision import OpenTeleVision
from Preprocessor import VuerPreprocessor
from constants_vuer import tip_indices
from dex_retargeting.retargeting_config import RetargetingConfig

from pathlib import Path
import time
import yaml
from multiprocessing import Process, shared_memory, Queue, Manager, Event, Lock

import cv2
import zmq
import pickle
import zlib

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from robot_control.robot_hand import H1HandController
from teleop.robot_control.robot_arm import H1ArmController, kNumMotors
from teleop.robot_control.robot_arm_ik import Arm_IK


def image_receiver(image_queue, resolution, crop_size_w, crop_size_h):
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.connect("tcp://192.168.123.162:5555")
    
    while True:
        compressed_data = b''
        while True:
            chunk = socket.recv()
            compressed_data += chunk
            if len(chunk) < 60000:
                break
        data = zlib.decompress(compressed_data)
        frame_data = pickle.loads(data)

        # Decode and display the image
        frame = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)
        sm.write_image(frame)
        # Control receiving frequency
        time.sleep(0.01)

class SharedMemoryImage:
    def __init__(self, img_shape):
        self.resolution = img_shape#(720, 1280)
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0]-self.crop_size_h, self.resolution[1] - 2 * self.crop_size_w)

        self.img_shape = (self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]

        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)
        self.lock = Lock()

    def write_image(self, image):
        with self.lock:
            np.copyto(self.img_array, image)

    def read_image(self):
        with self.lock:
            image_copy = self.img_array.copy()
            return image_copy

    def cleanup(self):
        self.shm.close()
        self.shm.unlink()

class VuerTeleop:
    def __init__(self, config_file_path):
        self.resolution = (480,640) #(720, 1280)
        self.crop_size_w = 0
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0]-self.crop_size_h, self.resolution[1]-2*self.crop_size_w)

        self.img_shape = (self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]

        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)
        image_queue = Queue()
        toggle_streaming = Event()
        self.tv = OpenTeleVision(self.resolution_cropped, self.shm.name, image_queue, toggle_streaming)
        self.processor = VuerPreprocessor()

        RetargetingConfig.set_default_urdf_dir('../assets')
        with Path(config_file_path).open('r') as f:
            cfg = yaml.safe_load(f)
        left_retargeting_config = RetargetingConfig.from_dict(cfg['left'])
        right_retargeting_config = RetargetingConfig.from_dict(cfg['right'])
        self.left_retargeting = left_retargeting_config.build()
        self.right_retargeting = right_retargeting_config.build()
    
    def step(self):
        head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process(self.tv)
        head_rmat = head_mat[:3, :3]

        left_wrist_mat[2, 3] +=0.45
        right_wrist_mat[2,3] +=0.45
        left_wrist_mat[0, 3] +=0.20
        right_wrist_mat[0,3] +=0.20

        left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]

        return head_rmat, left_wrist_mat, right_wrist_mat, left_qpos, right_qpos


if __name__ == '__main__':
    manager = Manager()
    image_queue = manager.Queue()
    teleoperator = VuerTeleop('inspire_hand.yml')
    h1hand = H1HandController()
    h1arm = H1ArmController()
    arm_ik = Arm_IK()
    sm = SharedMemoryImage((480,640))
    image_process = Process(target=image_receiver, args=(sm, teleoperator.resolution, teleoperator.crop_size_w, teleoperator.crop_size_h))
    image_process.start()
            
    try:
        user_input = input("Please enter the start signal (enter 's' to start the subsequent program):")
        if user_input.lower() == 's':
            while True:
                armstate = None
                armv = None 
                frame = sm.read_image()
                np.copyto(teleoperator.img_array, np.array(frame))
                handstate = h1hand.get_hand_state()

                q_poseList=np.zeros(kNumMotors) # h1 with 20 motors
                q_tau_ff=np.zeros(kNumMotors)
                armstate,armv = h1arm.GetMotorState()

                head_rmat, left_pose, right_pose, left_qpos, right_qpos = teleoperator.step()
                sol_q ,tau_ff, flag = arm_ik.ik_fun(left_pose, right_pose, armstate, armv)
                
                if flag:
                    q_poseList[12:20] = sol_q
                    q_tau_ff[12:20] = tau_ff
                else:
                    q_poseList[13:27] = armstate
                    q_tau_ff = np.zeros(20)

                h1arm.SetMotorPose(q_poseList, q_tau_ff)

                if right_qpos is not None and left_qpos is not None:
                    # 4,5: index 6,7: middle, 0,1: pinky, 2,3: ring, 8,9: thumb
                    right_angles = [1.7 - right_qpos[i] for i in [4, 6, 2, 0]]
                    right_angles.append(1.2 - right_qpos[8])
                    right_angles.append(0.5 - right_qpos[9])

                    left_angles = [1.7- left_qpos[i] for i in  [4, 6, 2, 0]]
                    left_angles.append(1.2 - left_qpos[8])
                    left_angles.append(0.5 - left_qpos[9])
                    h1hand.crtl(right_angles,left_angles)

    except KeyboardInterrupt:
        exit(0)
