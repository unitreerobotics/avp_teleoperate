import os
import time
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple
import numpy as np
from .agent import Agent
from .dynamixel_robot import DynamixelRobot
# from agent import Agent
# from dynamixel_robot import DynamixelRobot


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of dynamixel robot. Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of robot. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs is -1 for all dynamixel"""

    gripper_config: Tuple[int, int, int]
    """reserved for later work"""

    # it will run after init and work as init check
    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self, port: str = "/dev/ttyUSB0", start_joints: Optional[np.ndarray] = None
    ) -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            gripper_config=self.gripper_config,
            start_joints=start_joints,
        )

# Can put multi robot into the dic, note that the calibration info shoule be put here
PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    #! for camera mounta
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8IT033-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2),
        joint_offsets=(
            2*np.pi/2, 
            2*np.pi/2, 
        ),
        joint_signs=(-1, -1),
        gripper_config=None,
    ), 

}

# general we only input port into the class, other info is stored in the dic
class DynamixelAgent(Agent):
    def __init__(
        self,
        port: str,
        dynamixel_config: Optional[DynamixelRobotConfig] = None,
        start_joints: Optional[np.ndarray] = None,
        cap_num: int = 42,
    ):
        #! init dynamixel robot setting
        # use the config to make the robot
        if dynamixel_config is not None:
            self._robot = dynamixel_config.make_robot(
                port=port, start_joints=start_joints
            )
        # find the info auto
        else:
            # check port 
            assert os.path.exists(port), port
            assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"

            # use port to gain config
            config = PORT_CONFIG_MAP[port]
            self._robot = config.make_robot(port=port, start_joints=start_joints)

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray: 
        return self._robot.get_joint_state()
    
if __name__ == "__main__":
    agent = DynamixelAgent(port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8IST6E-if00-port0")

    agent._robot.set_torque_mode(True)

    min_radians = -1.57
    max_radians = 1.57
    interval = 0.1

    current_radian = 0
    while current_radian <= max_radians:
        action = agent.act(1)
        print("now action                     : ", [f"{x:.3f}" for x in action])
        command = [0, 0]
        agent._robot.command_joint_state([0, current_radian])
        time.sleep(0.1) 
        true_value = agent._robot._driver.get_joints()    
        print("true value                 : ", [f"{x:.3f}" for x in true_value])
        current_radian += interval
