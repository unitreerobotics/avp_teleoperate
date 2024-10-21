# this file is legacy, need to fix.
from unitree_dds_wrapper.idl import unitree_go
from unitree_dds_wrapper.publisher import Publisher
from unitree_dds_wrapper.subscription import Subscription
from avp_teleoperate.teleop.open_television.constants import inspire_tip_indices
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import numpy as np
import threading
import time

inspire_tip_indices = [4, 9, 14, 19, 24]

class InspireController:
    def __init__(self):
        self.cmd = unitree_go.msg.dds_.MotorCmds_()
        self.state = unitree_go.msg.dds_.MotorStates_()
        self.lock = threading.Lock()
        self.handcmd = Publisher(unitree_go.msg.dds_.MotorCmds_, "rt/inspire/cmd")
        self.handstate = Subscription(unitree_go.msg.dds_.MotorStates_, "rt/inspire/state")
        self.cmd.cmds = [unitree_go.msg.dds_.MotorCmd_() for _ in range(12)]
        self.state.states = [unitree_go.msg.dds_.MotorState_() for _ in range(12)]

        self.subscribe_state_thread = threading.Thread(target=self.subscribe_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)

    def subscribe_state(self):
        while True:
            if self.handstate.msg:
                self.state = self.handstate.msg
            time.sleep(0.01)

    def ctrl(self, left_angles, right_angles):
        for i in range(6):
            self.cmd.cmds[i].q = right_angles[i]
            self.cmd.cmds[i+6].q = left_angles[i]
        self.handcmd.msg.cmds = self.cmd.cmds
        self.handcmd.write()

    def get_current_dual_hand_q(self):
        with self.lock:
            q = np.array([self.state.states[i].q for i in range(12)])
            return q
        
    def get_right_q(self):
        with self.lock:
            q = np.array([self.state.states[i].q for i in range(6)])
            return q

    def get_left_q(self):
        with self.lock:
            q = np.array([self.state.states[i+6].q for i in range(6)])
            return q
