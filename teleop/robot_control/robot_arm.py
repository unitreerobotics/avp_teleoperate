import numpy as np
import threading
import time

from unitree_dds_wrapper.idl import unitree_hg
from unitree_dds_wrapper.publisher import Publisher
from unitree_dds_wrapper.subscription import Subscription
from unitree_dds_wrapper.utils.crc import crc32

import struct
from enum import IntEnum
import copy

kTopicLowCommand = "rt/lowcmd"
kTopicLowState = "rt/lowstate"
kNumMotors = 20



class MotorCommand:
    def __init__(self):
        self.q_ref = np.zeros(kNumMotors)  
        self.dq_ref = np.zeros(kNumMotors)  
        self.tau_ff = np.zeros(kNumMotors)  
        self.kp = np.zeros(kNumMotors)  
        self.kd = np.zeros(kNumMotors)  

class MotorState:
    def __init__(self):
        self.q = np.zeros(kNumMotors)
        self.dq = np.zeros(kNumMotors)

class BaseState:
    def __init__(self):
        self.omega = np.zeros(3)
        self.rpy = np.zeros(3)

class DataBuffer:
    def __init__(self):
        self.data = None
        self.lock = threading.Lock()

    def GetData(self):
        with self.lock:
            return self.data

    def SetData(self, data):
        with self.lock:
            self.data = data

np.set_printoptions(linewidth=240)

class H1ArmController:
    def __init__(self):
        print("Initialize H1ArmController...")
        self.q_desList = np.zeros(kNumMotors)
        self.q_tau_ff = np.zeros(kNumMotors)
        self.msg =  unitree_hg.msg.dds_.LowCmd_()
        self.__packFmtHGLowCmd = '<2B2x' + 'B3x5fI' * 35 + '5I'

        self.msg.head = [0xFE, 0xEF]
        self.lowcmd_publisher = Publisher(unitree_hg.msg.dds_.LowCmd_, kTopicLowCommand)
        self.lowstate_subscriber = Subscription(unitree_hg.msg.dds_.LowState_, kTopicLowState)

        self.motor_state_buffer = DataBuffer()
        self.motor_command_buffer = DataBuffer()
        self.base_state_buffer = DataBuffer()

        self.kp_low = 140.0
        self.kd_low = 7.5

        self.kp_high = 200.0
        self.kd_high = 5.0

        # self.kp_wrist = 35.0
        # self.kd_wrist = 6.0

        self.control_dt = 0.01
        self.hip_pitch_init_pos = -0.5
        self.knee_init_pos = 1.0
        self.ankle_init_pos = -0.5
        self.shoulder_pitch_init_pos = -1.4
        self.time = 0.0
        self.init_duration = 10.0
        self.report_dt = 0.1
        self.ratio = 0.0
        self.q_target = []
        while not self.lowstate_subscriber.msg:
            print("lowstate_subscriber is not ok! Please check dds.")
            time.sleep(0.01)
        
        for id in JointIndex:
            self.msg.motor_cmd[id].q = self.lowstate_subscriber.msg.motor_state[id].q
            self.q_target.append(self.msg.motor_cmd[id].q)
        print(f"Init q_pose is :{self.q_target}")
        duration = 1000
        init_q = np.array([self.lowstate_subscriber.msg.motor_state[id].q for id in JointIndex])
        print("Lock Leg...")
        for i in range(duration):
            time.sleep(0.001)
            q_t = init_q + (self.q_target - init_q) * i / duration
            for i, id in enumerate(JointIndex):
                self.msg.motor_cmd[id].mode = 1
                if id not in JointArmIndex:
                    self.msg.motor_cmd[id].kp = 200
                    self.msg.motor_cmd[id].kd = 5
                    self.msg.motor_cmd[id].q = q_t[i]
            self.pre_communication()
            self.lowcmd_publisher.msg = self.msg
            self.lowcmd_publisher.write()
        print("Lock Leg OK!")

        self.report_rpy_thread = threading.Thread(target=self.SubscribeState)
        self.report_rpy_thread.start()

        self.control_thread = threading.Thread(target=self.Control)
        self.control_thread.start()

        self.command_writer_thread = threading.Thread(target=self.LowCommandWriter)
        self.command_writer_thread.start()
        print("Initialize H1ArmController OK!")

    def LowStateHandler(self, message):
        low_state = message
        self.RecordMotorState(low_state)
        self.RecordBaseState(low_state)

    def SetMotorPose(self,q_desList,q_tau_ff):
        self.q_desList = q_desList
        self.q_tau_ff = q_tau_ff

    def __Trans(self, packData):
        calcData = []
        calcLen = ((len(packData)>>2)-1)

        for i in range(calcLen):
            d = ((packData[i*4+3] << 24) | (packData[i*4+2] << 16) | (packData[i*4+1] << 8) | (packData[i*4]))
            calcData.append(d)

        return calcData
    
    def __Crc32(self, data):
        bit = 0
        crc = 0xFFFFFFFF
        polynomial = 0x04c11db7

        for i in range(len(data)):
            bit = 1 << 31
            current = data[i]

            for b in range(32):
                if crc & 0x80000000:
                    crc = (crc << 1) & 0xFFFFFFFF
                    crc ^= polynomial
                else:
                    crc = (crc << 1) & 0xFFFFFFFF

                if current & bit:
                    crc ^= polynomial

                bit >>= 1
        
        return crc
    
    def pre_communication(self):
        self.__pack_crc()

    def __pack_crc(self):
        origData = []
        origData.append(self.msg.mode_pr)
        origData.append(self.msg.mode_machine)

        for i in range(kNumMotors):
            origData.append(self.msg.motor_cmd[i].mode)
            origData.append(self.msg.motor_cmd[i].q)
            origData.append(self.msg.motor_cmd[i].dq)
            origData.append(self.msg.motor_cmd[i].tau)
            origData.append(self.msg.motor_cmd[i].kp)
            origData.append(self.msg.motor_cmd[i].kd)
            origData.append(self.msg.motor_cmd[i].reserve)

        origData.extend(self.msg.reserve)
        origData.append(self.msg.crc)
        calcdata = struct.pack(self.__packFmtHGLowCmd, *origData)
        calcdata =  self.__Trans(calcdata)
        self.msg.crc = self.__Crc32(calcdata)

    def LowCommandWriter(self):
        while True:
            mc_tmp_ptr = self.motor_command_buffer.GetData()
            if mc_tmp_ptr:
                for i in JointArmIndex:
                    self.msg.motor_cmd[i].tau = mc_tmp_ptr.tau_ff[i]  
                    self.msg.motor_cmd[i].q = mc_tmp_ptr.q_ref[i]  
                    self.msg.motor_cmd[i].dq = mc_tmp_ptr.dq_ref[i]  
                    self.msg.motor_cmd[i].kp = mc_tmp_ptr.kp[i]  
                    self.msg.motor_cmd[i].kd = mc_tmp_ptr.kd[i]  
                self.pre_communication()
                self.lowcmd_publisher.msg = self.msg
                self.lowcmd_publisher.write()
            time.sleep(0.002)
                  
    def Control(self):
        while True:
            ms_tmp_ptr = self.motor_state_buffer.GetData()  
            if ms_tmp_ptr: 
                tem_q_desList = copy.deepcopy(self.q_desList)
                tem_q_tau_ff = copy.deepcopy(self.q_tau_ff)
                motor_command_tmp = MotorCommand()  
                self.time += self.control_dt  
                self.time = min(max(self.time, 0.0), self.init_duration)  
                self.ratio = self.time / self.init_duration  
                for i in range(kNumMotors):  
                    if self.IsWeakMotor(i):
                        motor_command_tmp.kp[i] = self.kp_low
                        motor_command_tmp.kd[i] = self.kd_low
                    # elif self.IsWristMotor(i):
                    #     motor_command_tmp.kp[i] = self.kp_wrist
                    #     motor_command_tmp.kd[i] = self.kd_wrist
                    else:
                        motor_command_tmp.kp[i] = self.kp_high
                        motor_command_tmp.kd[i] = self.kd_high
                    motor_command_tmp.dq_ref[i] = 0.0  
                    motor_command_tmp.tau_ff[i] = tem_q_tau_ff[i]  
                    q_des = tem_q_desList[i]
                    
                    q_des = (q_des - ms_tmp_ptr.q[i]) * self.ratio + ms_tmp_ptr.q[i]
                    motor_command_tmp.q_ref[i] = q_des 
                self.motor_command_buffer.SetData(motor_command_tmp)  
            time.sleep(0.002)
            
    def GetMotorState(self):
        ms_tmp_ptr = self.motor_state_buffer.GetData()
        if ms_tmp_ptr:
            return ms_tmp_ptr.q[13:27],ms_tmp_ptr.dq[13:27]
        else:
            return None,None

    def SubscribeState(self):
        while True:
            if self.lowstate_subscriber.msg:
                self.LowStateHandler(self.lowstate_subscriber.msg)
            time.sleep(0.002)

    def RecordMotorState(self, msg):
        ms_tmp = MotorState()
        for i in range(kNumMotors):
            ms_tmp.q[i] = msg.motor_state[i].q
            ms_tmp.dq[i] = msg.motor_state[i].dq
        self.motor_state_buffer.SetData(ms_tmp)

    def RecordBaseState(self, msg):
        bs_tmp = BaseState()
        bs_tmp.omega = msg.imu_state.gyroscope
        bs_tmp.rpy = msg.imu_state.rpy
        self.base_state_buffer.SetData(bs_tmp)

    def IsWeakMotor(self, motor_index):
        weak_motors = [
            JointIndex.kLeftAnkle,
            JointIndex.kRightAnkle,
            # Left arm
            JointIndex.kLeftShoulderPitch,
            JointIndex.kLeftShoulderRoll,
            JointIndex.kLeftShoulderYaw,
            JointIndex.kLeftElbow,
            # Right arm
            JointIndex.kRightShoulderPitch,
            JointIndex.kRightShoulderRoll,
            JointIndex.kRightShoulderYaw,
            JointIndex.kRightElbow,
        ]
        return motor_index in weak_motors
    
    # def IsWristMotor(self, motor_index):
    #     wrist_motors = [
    #         JointIndex.kLeftElbowRoll,
    #         JointIndex.kLeftWristPitch,
    #         JointIndex.kLeftWristyaw,
    #         JointIndex.kRightElbowRoll,
    #         JointIndex.kRightWristPitch,
    #         JointIndex.kRightWristYaw,
    #     ]
    #     return motor_index in wrist_motors

class JointArmIndex(IntEnum):
    # for H1
    # Right arm
    kRightShoulderPitch = 12
    kRightShoulderRoll = 13
    kRightShoulderYaw = 14
    kRightElbow = 15
    # Left arm
    kLeftShoulderPitch = 16
    kLeftShoulderRoll = 17
    kLeftShoulderYaw = 18
    kLeftElbow = 19

class JointIndex(IntEnum):
    # for H1
    # Right leg
    kRightHipYaw = 8
    kRightHipRoll = 0
    kRightHipPitch = 1
    kRightKnee = 2
    kRightAnkle = 11
    # Left leg
    kLeftHipYaw = 7
    kLeftHipRoll = 3
    kLeftHipPitch = 4
    kLeftKnee = 5
    kLeftAnkle = 10

    kWaistYaw = 6

    kNotUsedJoint = 9

    # Right arm
    kRightShoulderPitch = 12
    kRightShoulderRoll = 13
    kRightShoulderYaw = 14
    kRightElbow = 15
    # Left arm
    kLeftShoulderPitch = 16
    kLeftShoulderRoll = 17
    kLeftShoulderYaw = 18
    kLeftElbow = 19

