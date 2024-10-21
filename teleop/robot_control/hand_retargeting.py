from dex_retargeting.retargeting_config import RetargetingConfig
from pathlib import Path
import yaml
from enum import Enum

class HandType(Enum):
    INSPIRE_HAND = "../assets/inspire_hand/inspire_hand.yml"
    UNITREE_DEX3 = "../assets/unitree_hand/unitree_dex3.yml"

class HandRetargeting:
    def __init__(self, hand_type: HandType):
        RetargetingConfig.set_default_urdf_dir('../assets')

        config_file_path = hand_type.value

        with Path(config_file_path).open('r') as f:
            self.cfg = yaml.safe_load(f)

        left_retargeting_config = RetargetingConfig.from_dict(self.cfg['left'])
        right_retargeting_config = RetargetingConfig.from_dict(self.cfg['right'])
        self.left_retargeting = left_retargeting_config.build()
        self.right_retargeting = right_retargeting_config.build()