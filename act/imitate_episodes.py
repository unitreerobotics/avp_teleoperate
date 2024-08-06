import torch
import numpy as np
import os
import pickle
import argparse
import matplotlib.pyplot as plt
from copy import deepcopy
from tqdm import tqdm
from einops import rearrange

# from .constants import DT
# from .constants import PUPPET_GRIPPER_JOINT_OPEN
from utils import load_data # data functions
from utils import compute_dict_mean, set_seed, detach_dict, parse_id, find_all_ckpt # helper functions
from policy import ACTPolicy, CNNMLPPolicy
# from .visualize_episodes import save_videos
import wandb

# from sim_env import BOX_POSE
# from constants import SIM_TASK_CONFIGS
import IPython
e = IPython.embed
import time
from itertools import repeat

def repeater(data_loader):
    epoch = 0
    for loader in repeat(data_loader):
        for data in loader:
            yield data
        print(f'Epoch {epoch} done')
        epoch += 1

from pathlib import Path


def main(args):
    set_seed(1)
    # command line parameters
    is_eval = args['eval']
    policy_class = args['policy_class']
    onscreen_render = args['onscreen_render']
    # task_name = args['task_name']
    batch_size_train = args['batch_size']
    batch_size_val = args['batch_size']
    num_epochs = args['num_epochs']

    # get task parameters
    # is_sim = task_name[:4] == 'sim_'
    task_dir, task_name = parse_id(RECORD_DIR, args['taskid'])
    dataset_dir = (Path(task_dir) / 'processed').resolve()
    ckpt_dir = (LOG_DIR / task_name / args['exptid']).resolve()
    print("*"*20)
    print(f"Task name: {task_name}")
    print("*"*20)

    # print(f"Checkpoint dir: {ckpt_dir}")
    # task_config = SIM_TASK_CONFIGS[task_name]
    # dataset_dir = task_config['dataset_dir']
    # ckpt_dir = task_config['ckpt_dir']
    # num_episodes = task_config['num_episodes']
    # episode_len = task_config['episode_len']
    camera_names = ['left', 'right']

    # fixed parameters
    state_dim = 26
    action_dim = 28
    lr_backbone = 1e-5
    backbone = 'dino_v2'
    if policy_class == 'ACT':
        enc_layers = 4
        dec_layers = 7
        nheads = 8
        policy_config = {'lr': args['lr'],
                         'num_queries': args['chunk_size'],
                         'kl_weight': args['kl_weight'],
                         'hidden_dim': args['hidden_dim'],
                         'dim_feedforward': args['dim_feedforward'],
                         'lr_backbone': lr_backbone,
                         'backbone': backbone,
                         'enc_layers': enc_layers,
                         'dec_layers': dec_layers,
                         'nheads': nheads,
                         'camera_names': camera_names,
                         'state_dim': state_dim,
                         'action_dim': action_dim,
                         'qpos_noise_std': args['qpos_noise_std'],
                         }
    elif policy_class == 'CNNMLP':
        policy_config = {'lr': args['lr'], 'lr_backbone': lr_backbone, 'backbone' : backbone, 'num_queries': 1,
                         'camera_names': camera_names,}
    else:
        raise NotImplementedError

    config = {
        'num_epochs': num_epochs,
        'ckpt_dir': ckpt_dir,
        # 'episode_len': episode_len,
        'state_dim': state_dim,
        'action_dim': action_dim,
        'lr': args['lr'],
        'policy_class': policy_class,
        'onscreen_render': onscreen_render,
        'policy_config': policy_config,
        # 'task_name': task_name,
        'seed': args['seed'],
        'temporal_agg': args['temporal_agg'],
        'camera_names': camera_names,
        # 'real_robot': not is_sim
        'resumeid': args['resumeid'],
        'resume_ckpt': args['resume_ckpt'],
        'task_name': task_name,
        'exptid': args['exptid'],
    }
    mode = "disabled" if args["no_wandb"] or args["save_jit"] else "online"
    wandb.init(project="television", name=args['exptid'], group=task_name, entity="cxx", mode=mode, dir="../data/logs")
    wandb.config.update(config)
    train_dataloader, val_dataloader, stats, _ = load_data(dataset_dir, camera_names, batch_size_train, batch_size_val)

    # save dataset stats
    if not os.path.isdir(ckpt_dir):
        os.makedirs(ckpt_dir)
    stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
    with open(stats_path, 'wb') as f:
        pickle.dump(stats, f)

    if args['save_jit']:
        save_jit(config)
        return

    best_ckpt_info = train_bc(train_dataloader, val_dataloader, config)
    best_epoch, min_val_loss, best_state_dict = best_ckpt_info

    # save best checkpoint
    ckpt_path = os.path.join(ckpt_dir, f'policy_best.ckpt')
    torch.save(best_state_dict, ckpt_path)
    print(f'Best ckpt, val loss {min_val_loss:.6f} @ epoch{best_epoch}')
    wandb.finish()

def make_policy(policy_class, policy_config):
    if policy_class == 'ACT':
        policy = ACTPolicy(policy_config)
    elif policy_class == 'CNNMLP':
        policy = CNNMLPPolicy(policy_config)
    else:
        raise NotImplementedError
    return policy


def make_optimizer(policy_class, policy):
    if policy_class == 'ACT':
        optimizer = policy.configure_optimizers()
    elif policy_class == 'CNNMLP':
        optimizer = policy.configure_optimizers()
    else:
        raise NotImplementedError
    return optimizer


def get_image(ts, camera_names):
    curr_images = []
    for cam_name in camera_names:
        curr_image = rearrange(ts.observation['images'][cam_name], 'h w c -> c h w')
        curr_images.append(curr_image)
    curr_image = np.stack(curr_images, axis=0)
    curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)
    return curr_image


def forward_pass(data, policy):
    image_data, qpos_data, action_data, is_pad = data
    image_data, qpos_data, action_data, is_pad = image_data.cuda(), qpos_data.cuda(), action_data.cuda(), is_pad.cuda()
    return policy(qpos_data, image_data, action_data, is_pad) # TODO remove None



def train_bc(train_dataloader, val_dataloader, config):
    num_epochs = config['num_epochs']
    ckpt_dir = config['ckpt_dir']
    seed = config['seed']
    policy_class = config['policy_class']
    policy_config = config['policy_config']

    set_seed(seed)

    policy = make_policy(policy_class, policy_config)
    policy.cuda()
    optimizer = make_optimizer(policy_class, policy)

    if config['resumeid']:
        exp_dir, exp_name = parse_id((LOG_DIR / config['task_name']).resolve(), config['resumeid'])
        policy, _, _ = load_ckpt(policy, exp_dir, config['resume_ckpt'])
        # if config['resume_ckpt']:
        #     ckpt_name = f"policy_epoch_{config['resume_ckpt']}_seed_0.ckpt"
        # else:
        #     ckpt_name, _ = find_all_ckpt(exp_dir)#f"policy_last.ckpt"
        # resume_path = (Path(exp_dir) / ckpt_name).resolve()
        # print(f"Resuming from {resume_path}")
        # checkpoint = torch.load(resume_path)
        # policy.load_state_dict(checkpoint)

    # train_history = []
    # validation_history = []
    min_val_loss = np.inf
    best_ckpt_info = None

    train_dataloader = repeater(train_dataloader)
    for epoch in tqdm(range(num_epochs)):
        print(f'\nEpoch {epoch}')
        if epoch % 500 == 0:
        # validation
            with torch.inference_mode():
                policy.eval()
                validation_dicts = []
                for batch_idx, data in enumerate(val_dataloader):
                    forward_dict = forward_pass(data, policy)
                    validation_dicts.append(forward_dict)
                    if batch_idx > 20:
                        break

                validation_summary = compute_dict_mean(validation_dicts)
                
                epoch_val_loss = validation_summary['loss']
                if epoch_val_loss < min_val_loss:
                    min_val_loss = epoch_val_loss
                    best_ckpt_info = (epoch, min_val_loss, deepcopy(policy.state_dict()))
            for k in list(validation_summary.keys()):
                validation_summary[f'val/{k}'] = validation_summary.pop(k)            
            wandb.log(validation_summary, step=epoch)
            print(f'Val loss:   {epoch_val_loss:.5f}')
            summary_string = ''
            for k, v in validation_summary.items():
                summary_string += f'{k}: {v.item():.3f} '
            print(summary_string)

        # training
        policy.train()
        optimizer.zero_grad()
        
        data = next(train_dataloader)
        forward_dict = forward_pass(data, policy)
        # backward
        loss = forward_dict['loss']
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()
        
        epoch_summary = detach_dict(forward_dict)

        # epoch_summary = compute_dict_mean(train_history[(batch_idx+1)*epoch:(batch_idx+1)*(epoch+1)])
        epoch_train_loss = epoch_summary['loss']
        print(f'Train loss: {epoch_train_loss:.5f}')
        summary_string = ''
        for k, v in epoch_summary.items():
            summary_string += f'{k}: {v.item():.3f} '
        print(summary_string)
        wandb.log(epoch_summary, step=epoch)

        if epoch % 1000 == 0 and epoch >= 1000:
            ckpt_path = os.path.join(ckpt_dir, f'policy_epoch_{epoch}_seed_{seed}.ckpt')
            torch.save(policy.state_dict(), ckpt_path)
            # plot_history(train_history, validation_history, epoch, ckpt_dir, seed)

    ckpt_path = os.path.join(ckpt_dir, f'policy_last.ckpt')
    torch.save(policy.state_dict(), ckpt_path)

    best_epoch, min_val_loss, best_state_dict = best_ckpt_info
    ckpt_path = os.path.join(ckpt_dir, f'policy_epoch_{best_epoch}_seed_{seed}.ckpt')
    torch.save(best_state_dict, ckpt_path)
    print(f'Training finished:\nSeed {seed}, val loss {min_val_loss:.6f} at epoch {best_epoch}')

    # save training curves
    # plot_history(train_history, validation_history, num_epochs, ckpt_dir, seed)

    return best_ckpt_info


def plot_history(train_history, validation_history, num_epochs, ckpt_dir, seed):
    # save training curves
    for key in train_history[0]:
        plot_path = os.path.join(ckpt_dir, f'train_val_{key}_seed_{seed}.png')
        plt.figure()
        train_values = [summary[key].item() for summary in train_history]
        val_values = [summary[key].item() for summary in validation_history]
        plt.plot(np.linspace(0, num_epochs-1, len(train_history)), train_values, label='train')
        plt.plot(np.linspace(0, num_epochs-1, len(validation_history)), val_values, label='validation')
        # plt.ylim([-0.1, 1])
        plt.tight_layout()
        plt.legend()
        plt.title(key)
        plt.savefig(plot_path)
    print(f'Saved plots to {ckpt_dir}')

def load_ckpt(policy, exp_dir, ckpt_name):
    if ckpt_name:
        epoch = ckpt_name
        ckpt_name = f"policy_epoch_{ckpt_name}_seed_0.ckpt"
    else:
        ckpt_name, epoch = find_all_ckpt(exp_dir)#f"policy_last.ckpt"
    resume_path = (Path(exp_dir) / ckpt_name).resolve()
    print("*"*20)
    print(f"Resuming from {resume_path}")
    print("*"*20)
    policy.load_state_dict(torch.load(resume_path))
    return policy, ckpt_name, epoch

def save_jit(config):
    # ckpt_dir = config['ckpt_dir']
    policy_class = config['policy_class']
    policy_config = config['policy_config']

    exp_dir, exp_name = parse_id((LOG_DIR / config['task_name']).resolve(), config['exptid'])

    policy = make_policy(policy_class, policy_config)
    policy.cuda()
    
    policy, ckpt_name, epoch = load_ckpt(policy, exp_dir, config['resume_ckpt'])

    policy.eval()
    image_data = torch.rand((1, 2, 3, 480, 640), device='cuda')
    qpos_data = torch.rand((1, config['state_dim']), device='cuda')
    input_data = (qpos_data, image_data)

    traced_policy = torch.jit.trace(policy, input_data)
    save_path = os.path.join(exp_dir, f"traced_jit_{epoch}.pt")
    traced_policy.save(save_path)
    print("Saved traced actor at ", save_path)

    new_policy = torch.jit.load(save_path)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval', action='store_true')
    parser.add_argument('--onscreen_render', action='store_true')
    parser.add_argument('--policy_class', action='store', type=str, help='policy_class, capitalize', required=True)
    # parser.add_argument('--task_name', action='store', type=str, help='task_name', required=True)
    parser.add_argument('--batch_size', action='store', type=int, help='batch_size', required=True)
    parser.add_argument('--seed', action='store', type=int, help='seed', required=True)
    parser.add_argument('--num_epochs', action='store', type=int, help='num_epochs', required=True)
    parser.add_argument('--lr', action='store', type=float, help='lr', required=True)
    parser.add_argument('--qpos_noise_std', action='store', default=0, type=float, help='lr', required=False)

    # for ACT
    parser.add_argument('--kl_weight', action='store', type=int, help='KL Weight', required=False)
    parser.add_argument('--chunk_size', action='store', type=int, help='chunk_size', required=False)
    parser.add_argument('--hidden_dim', action='store', type=int, help='hidden_dim', required=False)
    parser.add_argument('--dim_feedforward', action='store', type=int, help='dim_feedforward', required=False)
    parser.add_argument('--temporal_agg', action='store_true')
    parser.add_argument('--save_jit', action='store_true')
    parser.add_argument('--no_wandb', action='store_true')
    parser.add_argument('--resumeid', action='store', default="", type=str, help='resume id', required=False)
    parser.add_argument('--resume_ckpt', action='store', default="", type=str, help='resume ckpt', required=False)
    parser.add_argument('--taskid', action='store', type=str, help='task id', required=True)
    parser.add_argument('--exptid', action='store', type=str, help='experiment id', required=True)
    parser.add_argument('--source', choices=['self', 'ssd'], default='self')
    args = vars(parser.parse_args())

    if args['source'] == 'self':
        current_dir = Path(__file__).parent.resolve()
    else:
        current_dir = Path("/media/cxx/Extreme Pro/human2robot/data/").resolve()
    DATA_DIR = (current_dir.parent / 'data/').resolve()
    RECORD_DIR = (DATA_DIR / 'recordings/').resolve()
    LOG_DIR = (DATA_DIR / 'logs/').resolve()
    # print(f"\nDATA dir: {DATA_DIR}")

    main(args)
