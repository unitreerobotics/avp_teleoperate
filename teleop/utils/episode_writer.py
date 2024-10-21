import os
import cv2
import json
import datetime
import numpy as np
import time


class EpisodeWriter(object):
    def __init__(self, task_dir, frequency=30, image_size=[640, 480]):
        """
        image_size: [width, height]
        """
        print("==> EpisodeWriter initializing...\n")
        self.task_dir = task_dir
        self.frequency = frequency
        self.image_size = image_size
        
        self.data = {}
        self.episode_data = []
        self.item_id = -1
        self.episode_id = -1
        if os.path.exists(self.task_dir):
            episode_dirs = [episode_dir for episode_dir in os.listdir(self.task_dir) if 'episode_' in episode_dir]
            episode_last = sorted(episode_dirs)[-1] if len(episode_dirs) > 0 else None
            self.episode_id = 0 if episode_last is None else int(episode_last.split('_')[-1])
            print(f"==> task_dir directory already exist, now self.episode_id is：{self.episode_id}\n")
        else:
            os.makedirs(self.task_dir)
            print(f"==> episode directory does not exist, now create one.\n")
        self.data_info()
        self.text_desc()
        print("==> EpisodeWriter initialized successfully.\n")

    def data_info(self, version='1.0.0', date=None, author=None):
        self.info = {
                "version": "1.0.0" if version is None else version, 
                "date": "" if date is None else datetime.date.today().strftime('%Y-%m-%d'), 
                "author": "unitree" if author is None else author,
                "image": {"width":self.image_size[0], "height":self.image_size[1], "fps":self.frequency},
                "depth": {"width":self.image_size[0], "height":self.image_size[1], "fps":self.frequency},
                "audio": {"sample_rate": 16000, "channels": 1, "format":"PCM", "bits":16},    # PCM_S16
                "joint_names":{
                    "left_arm":   ['kLeftShoulderPitch' ,'kLeftShoulderRoll', 'kLeftShoulderYaw', 'kLeftElbow', 'kLeftWristRoll', 'kLeftWristPitch', 'kLeftWristyaw'],
                    "left_hand":  [],
                    "right_arm":  [],
                    "right_hand": [],
                    "body":       [],
                },

                "tactile_names": {
                    "left_hand": [],
                    "right_hand": [],
                }, 
            }
    def text_desc(self):
        self.text = {
            "goal": "Pick up the red cup on the table.",
            "desc": "Pick up the cup from the table and place it in another position. The operation should be smooth and the water in the cup should not spill out",
            "steps":"step1: searching for cups. step2: go to the target location. step3: pick up the cup",
        }

 
    def create_episode(self):
        """
        Create a new episode, each episode needs to specify the episode_id.
            text: Text descriptions of operation goals, steps, etc. The text description of each episode is the same.
            goal: operation goal
            desc: description
            steps: operation steps
        """

        self.item_id = -1
        self.episode_data = []
        self.episode_id = self.episode_id + 1
        
        self.episode_dir = os.path.join(self.task_dir, f"episode_{str(self.episode_id).zfill(4)}")
        self.color_dir = os.path.join(self.episode_dir, 'colors')
        self.depth_dir = os.path.join(self.episode_dir, 'depths')
        self.audio_dir = os.path.join(self.episode_dir, 'audios')
        self.json_path = os.path.join(self.episode_dir, 'data.json')
        os.makedirs(self.episode_dir, exist_ok=True)
        os.makedirs(self.color_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.audio_dir, exist_ok=True)
        
    def add_item(self, colors, depths=None, states=None, actions=None, tactiles=None, audios=None, log = True):
        self.item_id += 1
        item_data = {
            'idx': self.item_id,
            'colors': {"color_0": None},       
            'depths': {"depth_0": None},
            'states': {'left_arm':{}, 'right_arm':{}, 'left_hand':{}, 'right_hand':{}, 'body': {}},
            'actions': {'left_arm':{}, 'right_arm':{}, 'left_hand':{}, 'right_hand':{}, 'body': {}},
            'tactiles': {'left_hand':[], 'right_hand':[]},
            'audios': { "mic0": None},
        }

        # save images
        if colors is not None:
            for idx, (_, color) in enumerate(colors.items()):
                color_key = f'color_{idx}'
                color_name = f'{str(self.item_id).zfill(6)}_{color_key}.jpg'
                cv2.imwrite(os.path.join(self.color_dir, color_name), color)
                item_data['colors'][color_key] = os.path.join('colors', color_name)

        # save depths
        if depths is not None:
            for idx, (_, depth) in enumerate(depths.items()):
                depth_key = f'depth_{idx}'
                depth_name = f'{str(self.item_id).zfill(6)}_{depth_key}.png'
                cv2.imwrite(os.path.join(self.depth_dir, depth_name), depth)
                item_data['depths'][depth_key] = os.path.join('depths', depth_name)

        # save audios
        if audios is not None:
            for mic, audio in audios.items():
                audio_name = f'audio_{str(self.item_id).zfill(6)}_{mic}.npy'
                np.save(os.path.join(self.audio_dir, audio_name), audio.astype(np.int16))
                item_data['audios'][mic] = os.path.join('audios', audio_name)

        item_data['states'] = states
        item_data['actions'] = actions
        item_data['tactiles'] = tactiles

        self.episode_data.append(item_data)

        if log:
            curent_record_time = time.time()
            print(f"==> episode_id:{self.episode_id}  item_id:{self.item_id}  current_time:{curent_record_time}")


    def save_episode(self):
        """
            with open("./hmm.json",'r',encoding='utf-8') as json_file:
                model=json.load(json_file)
        """
        self.data['info'] = self.info
        self.data['text'] = self.text
        self.data['data'] = self.episode_data
        with open(self.json_path,'w',encoding='utf-8') as jsonf:
            jsonf.write(json.dumps(self.data, indent=4, ensure_ascii=False))