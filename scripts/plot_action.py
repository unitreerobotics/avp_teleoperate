import math
import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path
import h5py
from tqdm import tqdm
import time
import yaml
import pickle


if __name__ == '__main__':

    root = "../data/recordings"
    exp_name = "00-can-sorting"
    episode_name = "processed_episode_0.hdf5"
    episode_path = Path(root) / exp_name / "processed" / episode_name

    data = h5py.File(str(episode_path), 'r')
    actions = np.array(data['qpos_action'])
    data.close()
    timestamps = actions.shape[0]
    action_dim = actions.shape[1]

    plot_num = np.ceil(np.sqrt(action_dim)).astype(int)
    plt.subplot(plot_num, plot_num, 1)

    for i in range(action_dim):
        plt.subplot(plot_num, plot_num, i+1)
        plt.plot(np.arange(timestamps), actions[:,i])

    plt.show()
