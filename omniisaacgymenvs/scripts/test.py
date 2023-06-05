# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import numpy as np
import torch
import hydra
from omegaconf import DictConfig

from omniisaacgymenvs.utils.hydra_cfg.hydra_utils import *
from omniisaacgymenvs.utils.hydra_cfg.reformat import omegaconf_to_dict, print_dict

from omniisaacgymenvs.utils.task_util import initialize_task
from omniisaacgymenvs.envs.vec_env_rlgames import VecEnvRLGames

import rospy
from std_msgs.msg import Float32, Int16, String


def name_cb(msg, _params_dict):
    if len(_params_dict['joint_names']) == 0:
        return
    if '_' in msg.data:
        tmp = msg.data.split('_')
        data = f'{tmp[0].lower()}_{tmp[1].upper()}'
    else:
        data = msg.data
    _params_dict['joint_name'] = data
    _params_dict['changed'] = True
    # print(f'joint name: {joint_name}\tid: {joint_id}\tvalue: {joint_value}')


def val_cb(msg, _params_dict):
    if len(_params_dict['joint_names']) == 0:
        return
    _params_dict['joint_value'] = msg.data
    _params_dict['changed'] = True
    print(
        f'joint name: {_params_dict["joint_name"]}'
        f'\tid: {_params_dict["joint_id"]}'
        f'\tvalue: {_params_dict["joint_value"]}')


@hydra.main(config_name="config", config_path="../cfg")
def parse_hydra_configs(cfg: DictConfig):
    params_dict = {}
    params_dict['changed'] = False
    params_dict['joint_id'] = -1
    params_dict['joint_value'] = 0
    params_dict['joint_names'] = []
    params_dict['joint_name'] = 'all'

    rospy.init_node('a')
    name_sub = rospy.Subscriber('/joint_name', String, name_cb, (params_dict))
    val_sub = rospy.Subscriber('/joint_value', Float32, val_cb, (params_dict))

    cfg_dict = omegaconf_to_dict(cfg)
    print_dict(cfg_dict)

    headless = cfg.headless
    render = not headless
    enable_viewport = "enable_cameras" in cfg.task.sim and cfg.task.sim.enable_cameras

    env = VecEnvRLGames(headless=headless, sim_device=cfg.device_id, enable_livestream=cfg.enable_livestream, enable_viewport=enable_viewport)
    # sets seed. if seed is -1 will pick a random one
    from omni.isaac.core.utils.torch.maths import set_seed
    from omni.isaac.core.utils.torch import unscale
    cfg.seed = set_seed(cfg.seed, torch_deterministic=cfg.torch_deterministic)
    cfg_dict['seed'] = cfg.seed

    rospy.sleep(5)
    task = initialize_task(cfg_dict, env, no_obj_grav=True)
    rospy.sleep(5)
    first_loop = True
    params_dict['joint_names'] = env._task._hands.actuated_joint_names
    dof_limits = env._task._hands.get_dof_limits()
    dof_limits[:, :, 0] = dof_limits[:, :, 0] + 0.07
    dof_limits[:, :, 1] = dof_limits[:, :, 1] - 0.07
    hand_dof_lower_limits, hand_dof_upper_limits = torch.t(dof_limits[0].to(task.rl_device))
    i = 0
    a = False
    while True:
        if i <= 110:
            env._world.step(render=render)
        if i == 100:
            sample = env.action_space.sample()
            action_tensor = np.resize(np.zeros_like(sample), (env.num_envs, sample.shape[0]))
            actions = torch.tensor(action_tensor, device=task.rl_device)
        if i > 110:
            env._task.pre_physics_step(actions)
            env._world.step(render=render)
            env.sim_frame_count += 1
            env._task.post_physics_step()
        print(i)
        i += 1
        if i > 150:
            a = True
            break
        if a:
            break

    while env._simulation_app.is_running():
        if env._world.is_playing():
            if env._world.current_time_step_index == 0:
                env._world.reset(soft=True)
            # actions = torch.tensor(np.array([env.action_space.sample() for _ in range(env.num_envs)]), device=task.rl_device)
            if first_loop:
                sample = env.action_space.sample()
                first_loop = False
                action_tensor = np.resize(np.zeros_like(sample), (env.num_envs, sample.shape[0]))
            if params_dict['joint_name'] == 'all':
                action_tensor[:, :] = params_dict['joint_value']
            else:
                if params_dict['changed']:
                    params_dict['joint_id'] = env._task._hands.get_dof_index(params_dict['joint_name'])
                    print(
                        f'joint name: {params_dict["joint_name"]}'
                        f'\tid: {params_dict["joint_id"]}'
                        f'\tvalue: {params_dict["joint_value"]}')
                    params_dict['changed'] = False
                hand_joint_index = params_dict['joint_id']
                try:
                    index_in_action_tensor = env._task._hands.actuated_dof_indices.index(hand_joint_index)
                except ValueError as e:
                    print(f'joint index {hand_joint_index} could not be found in actuated indices list')
                else:
                    action_tensor[:, index_in_action_tensor] = params_dict['joint_value']
            actions = torch.tensor(action_tensor, device=task.rl_device)
            actions = unscale(actions,
                              hand_dof_lower_limits[env._task._hands.actuated_dof_indices],
                              hand_dof_upper_limits[env._task._hands.actuated_dof_indices])
            env._task.pre_physics_step(actions)
            env._world.step(render=render)
            env.sim_frame_count += 1
            env._task.post_physics_step()
        else:
            env._world.step(render=render)

    env._simulation_app.close()

if __name__ == '__main__':
    parse_hydra_configs()
