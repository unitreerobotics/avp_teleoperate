from typing import Dict, Optional, Sequence, Tuple

import numpy as np

# from robots.robot import Robot
import sys
from .robot import Robot
from .driver import DynamixelDriver,DynamixelDriverProtocol,FakeDynamixelDriver
# from robot import Robot
# from driver import DynamixelDriver,DynamixelDriverProtocol,FakeDynamixelDriver


class DynamixelRobot(Robot):
    """A class representing a UR robot."""

    def __init__(
        self,
        joint_ids: Sequence[int],
        joint_offsets: Optional[Sequence[float]] = None,
        joint_signs: Optional[Sequence[int]] = None,
        real: bool = False,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        gripper_config: Optional[Tuple[int, float, float]] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        

        print(f"attempting to connect to port: {port}")

        # used to set gripper, reserved for later
        self.gripper_open_close: Optional[Tuple[float, float]]

        if gripper_config is not None:
            assert joint_offsets is not None
            assert joint_signs is not None

            joint_ids = tuple(joint_ids) + (gripper_config[0],)
            joint_offsets = tuple(joint_offsets) + (0.0,)
            joint_signs = tuple(joint_signs) + (1,)
            self.gripper_open_close = (
                gripper_config[1] * np.pi / 180,
                gripper_config[2] * np.pi / 180,
            )
        else:
            self.gripper_open_close = None

        # all the self variable will start with _

        # set joint config
        self._joint_ids = joint_ids
        self._driver: DynamixelDriverProtocol

        if joint_offsets is None:
            self._joint_offsets = np.zeros(len(joint_ids))
        else:
            self._joint_offsets = np.array(joint_offsets)

        if joint_signs is None:
            self._joint_signs = np.ones(len(joint_ids))
        else:
            self._joint_signs = np.array(joint_signs)

        # check
        assert len(self._joint_ids) == len(self._joint_offsets), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_offsets: {len(self._joint_offsets)}"
        )
        assert len(self._joint_ids) == len(self._joint_signs), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_signs: {len(self._joint_signs)}"
        )
        assert np.all(
            np.abs(self._joint_signs) == 1
        ), f"joint_signs: {self._joint_signs}"

        # when called in gello_agent, real == True, used to do test
        if real:
            self._driver = DynamixelDriver(joint_ids, port=port, baudrate=baudrate)
            # we dault set the torque_mode(False)
            self._driver.set_torque_mode(False)
        else:
            self._driver = FakeDynamixelDriver(joint_ids)
            
        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.99

        #! this part might important! need to check if the robot start at different joints 
        if start_joints is not None:
            # loop through all joints and add +- 2pi to the joint offsets to get the closest to start joints
            new_joint_offsets = []
            current_joints = self.get_joint_state()
            assert current_joints.shape == start_joints.shape

            if gripper_config is not None:
                current_joints = current_joints[:-1]
                start_joints = start_joints[:-1]

            for c_joint, s_joint, joint_offset in zip(
                current_joints, start_joints, self._joint_offsets
            ):
                new_joint_offsets.append(
                    np.pi * 2 * np.round((s_joint - c_joint) / (2 * np.pi))
                    + joint_offset
                )

            if gripper_config is not None:
                new_joint_offsets.append(self._joint_offsets[-1])
            self._joint_offsets = np.array(new_joint_offsets)


    #! might change to @property
    def num_dofs(self) -> int:
        return len(self._joint_ids)

    #! now the joint num is different
    def get_joint_state(self) -> np.ndarray:
        pos = (self._driver.get_joints() - self._joint_offsets) * self._joint_signs
        assert len(pos) == self.num_dofs()

        if self.gripper_open_close is not None:
            # map pos to [0, 1]
            g_pos = (pos[-1] - self.gripper_open_close[0]) / (
                self.gripper_open_close[1] - self.gripper_open_close[0]
            )
            g_pos = min(max(0, g_pos), 1)
            pos[-1] = g_pos

        if self._last_pos is None:
            self._last_pos = pos
        else:
            # exponential smoothing
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos

        new_pos = np.append(pos, 0)
        return new_pos

    def map_to_valid_range(self, radians_array):
        mapped_radians = np.mod(radians_array, 2 * np.pi)
        return mapped_radians

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        # print("command                   : ", [f"{x:.3f}" for x in joint_state])
        set_value = (joint_state + self._joint_offsets).tolist()
        # print("_set value                 : ", [f"{x:.3f}" for x in set_value])
        set_value = self.map_to_valid_range(set_value)
        # print("set value                 : ", [f"{x:.3f}" for x in set_value])
        self._driver.set_joints(set_value)

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def get_observations(self) -> Dict[str, np.ndarray]:
        return {"joint_state": self.get_joint_state()}
    

