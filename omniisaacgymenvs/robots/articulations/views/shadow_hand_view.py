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

import os
from typing import Optional

from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView

import torch

class ShadowHandView(ArticulationView):
    def __init__(
        self,
        prim_paths_expr: str,
        name: Optional[str] = "ShadowHandView",
    ) -> None:

        super().__init__(
            prim_paths_expr=prim_paths_expr,
            name=name,
            reset_xform_properties=False
        )
        mujoco_env_str = os.environ.get('USE_MUJOCO')
        if mujoco_env_str is None:
            self.mujoco = True
        elif 'true' in mujoco_env_str.lower():
            self.mujoco = True
        elif 'false' in mujoco_env_str.lower():
            self.mujoco = False
        if self.mujoco:
            self._fingers = RigidPrimView(prim_paths_expr="/World/envs/.*/shadow_hand/lh.*distal", name="finger_view", reset_xform_properties=False)
        else:
            self._fingers = RigidPrimView(prim_paths_expr="/World/envs/.*/shadow_hand/robot0.*distal", name="finger_view", reset_xform_properties=False)

    @property
    def actuated_dof_indices(self):
        return self._actuated_dof_indices

    def initialize(self, physics_sim_view):
        super().initialize(physics_sim_view)
        if self.mujoco:
            side = 'lh'
            j0_name = 'J2'
            self.actuated_joint_names = [f'{side}_WRJ2', f'{side}_WRJ1',
                                         f'{side}_FFJ4', f'{side}_FFJ3', f'{side}_FF{j0_name}',
                                         f'{side}_MFJ4', f'{side}_MFJ3', f'{side}_MF{j0_name}',
                                         f'{side}_RFJ4', f'{side}_RFJ3', f'{side}_RF{j0_name}',
                                         f'{side}_LFJ5', f'{side}_LFJ4', f'{side}_LFJ3', f'{side}_LF{j0_name}',
                                         f'{side}_THJ5', f'{side}_THJ4', f'{side}_THJ3', f'{side}_THJ2', f'{side}_THJ1',
                                        ]
        else:

            self.actuated_joint_names = ['robot0_WRJ1', 'robot0_WRJ0',
                                         'robot0_FFJ3', 'robot0_FFJ2', 'robot0_FFJ1',
                                         'robot0_MFJ3', 'robot0_MFJ2', 'robot0_MFJ1',
                                         'robot0_RFJ3', 'robot0_RFJ2', 'robot0_RFJ1',
                                         'robot0_LFJ4', 'robot0_LFJ3', 'robot0_LFJ2', 'robot0_LFJ1',
                                         'robot0_THJ4', 'robot0_THJ3', 'robot0_THJ2', 'robot0_THJ1', 'robot0_THJ0',
                                         ]
        self._actuated_dof_indices = list()
        for joint_name in self.actuated_joint_names:
            self._actuated_dof_indices.append(self.get_dof_index(joint_name))
        self._actuated_dof_indices.sort()

        limit_stiffness = torch.tensor([30.0] * self.num_fixed_tendons, device=self._device)
        damping = torch.tensor([0.1] * self.num_fixed_tendons, device=self._device)
        self.set_fixed_tendon_properties(dampings=damping, limit_stiffnesses=limit_stiffness)
