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


from typing import Optional

import torch
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.utils.prims import get_prim_at_path, get_all_matching_child_prims, get_prim_children
import os
os.environ["USE_MUJOCO"] = "true"

class ShadowHandView(ArticulationView):
    def __init__(
        self,
        prim_paths_expr: str,
        name: Optional[str] = "ShadowHandView",
    ) -> None:
        super().__init__(prim_paths_expr=prim_paths_expr, name=name, reset_xform_properties=False)

        mujoco_env_str = os.environ.get('USE_MUJOCO')
        if mujoco_env_str is None:
            self.mujoco = True
        elif 'true' in mujoco_env_str.lower():
            self.mujoco = True
        elif 'false' in mujoco_env_str.lower():
            self.mujoco = False
        if not self.mujoco:
            self._hand_joint_prefix = 'robot0'
            prim_paths_expr = "/World/envs/.*/shadow_hand/robot0.*distal"
        else:
            self._side = 'rh'
            self._hand_joint_prefix = f'{self._side}'
            prim_paths_expr = f"/World/envs/.*/right_hand/rh_forearm/{self._side}.*distal"

        self._fingers = RigidPrimView(
            prim_paths_expr=prim_paths_expr,
            name="finger_view",
            reset_xform_properties=False,
        )

    @property
    def actuated_dof_indices(self):
        return self._actuated_dof_indices

    def initialize(self, physics_sim_view):
        super().initialize(physics_sim_view)
        if not self.mujoco:
            self.actuated_joint_names = [
                f"{self._hand_joint_prefix}_WRJ1",
                f"{self._hand_joint_prefix}_WRJ0",
                f"{self._hand_joint_prefix}_FFJ3",
                f"{self._hand_joint_prefix}_FFJ2",
                f"{self._hand_joint_prefix}_FFJ1",
                f"{self._hand_joint_prefix}_MFJ3",
                f"{self._hand_joint_prefix}_MFJ2",
                f"{self._hand_joint_prefix}_MFJ1",
                f"{self._hand_joint_prefix}_RFJ3",
                f"{self._hand_joint_prefix}_RFJ2",
                f"{self._hand_joint_prefix}_RFJ1",
                f"{self._hand_joint_prefix}_LFJ4",
                f"{self._hand_joint_prefix}_LFJ3",
                f"{self._hand_joint_prefix}_LFJ2",
                f"{self._hand_joint_prefix}_LFJ1",
                f"{self._hand_joint_prefix}_THJ4",
                f"{self._hand_joint_prefix}_THJ3",
                f"{self._hand_joint_prefix}_THJ2",
                f"{self._hand_joint_prefix}_THJ1",
                f"{self._hand_joint_prefix}_THJ0",
            ]
        else:
            j0_name = 'J2'
            self.actuated_joint_names = [
                f"{self._hand_joint_prefix}_WRJ2",
                f"{self._hand_joint_prefix}_WRJ1",
                f"{self._hand_joint_prefix}_FFJ4",
                f"{self._hand_joint_prefix}_FFJ3",
                f"{self._hand_joint_prefix}_FF{j0_name}",
                f"{self._hand_joint_prefix}_MFJ4",
                f"{self._hand_joint_prefix}_MFJ3",
                f"{self._hand_joint_prefix}_MF{j0_name}",
                f"{self._hand_joint_prefix}_RFJ4",
                f"{self._hand_joint_prefix}_RFJ3",
                f"{self._hand_joint_prefix}_RF{j0_name}",
                f"{self._hand_joint_prefix}_LFJ5",
                f"{self._hand_joint_prefix}_LFJ4",
                f"{self._hand_joint_prefix}_LFJ3",
                f"{self._hand_joint_prefix}_LF{j0_name}",
                f"{self._hand_joint_prefix}_THJ5",
                f"{self._hand_joint_prefix}_THJ4",
                f"{self._hand_joint_prefix}_THJ3",
                f"{self._hand_joint_prefix}_THJ2",
                f"{self._hand_joint_prefix}_THJ1",
            ]
        self._actuated_dof_indices = list()
        for joint_name in self.actuated_joint_names:
            self._actuated_dof_indices.append(self.get_dof_index(joint_name))
        self._actuated_dof_indices.sort()

        limit_stiffness = torch.tensor([30.0] * self.num_fixed_tendons, device=self._device)
        damping = torch.tensor([0.1] * self.num_fixed_tendons, device=self._device)
        self.set_fixed_tendon_properties(dampings=damping, limit_stiffnesses=limit_stiffness)

        fingertips = [f"{self._hand_joint_prefix}_ffdistal",
                      f"{self._hand_joint_prefix}_mfdistal",
                      f"{self._hand_joint_prefix}_rfdistal",
                      f"{self._hand_joint_prefix}_lfdistal",
                      f"{self._hand_joint_prefix}_thdistal"]
        self._sensor_indices = torch.tensor([self._body_indices[j] for j in fingertips], device=self._device, dtype=torch.long)

