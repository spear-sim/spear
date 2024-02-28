#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np


def modify_observation_for_visualization(obs, observation_comoponents_to_modify):
    
    modified_obs = {}
    for render_pass, camera_names in observation_comoponents_to_modify.items():
        for camera_name in camera_names:
            obs_render_pass = obs[camera_name]
            assert len(obs_render_pass.shape) == 3  # width, height, #channels
            assert obs_render_pass.shape[2] == 4    # 4 channels (RGBA)

            if render_pass == "depth":
                obs_render_pass_vis = obs_render_pass[:,:,[0,1,2]].copy() # depth is returned as RGBA

                # discard very large depth values
                max_depth_meters = 20.0
                obs_render_pass_vis = obs_render_pass_vis[:,:,0]
                obs_render_pass_vis = np.clip(obs_render_pass_vis, 0.0, max_depth_meters)

            elif render_pass == "final_color":
                obs_render_pass_vis = obs_render_pass[:,:,[2,1,0]].copy() # final_color is returned as BGRA

            elif render_pass == "normal":
                obs_render_pass_vis = obs_render_pass[:,:,[0,1,2]].copy() # normal is returned as RGBA

                # discard normals that aren't properly normalized, i.e., length of 1.0
                discard_mask = np.logical_not(np.isclose(np.linalg.norm(obs_render_pass_vis, axis=2), 1.0, rtol=0.001, atol=0.001))
                obs_render_pass_vis = np.clip((obs_render_pass_vis + 1.0) / 2.0, 0.0, 1.0)
                obs_render_pass_vis[discard_mask] = np.nan

            elif render_pass == "segmentation":
                obs_render_pass_vis = obs_render_pass[:,:,[2,1,0]].copy() # segmentation is returned as BGRA

            else:
                assert False
        
            modified_obs[camera_name] = obs_render_pass_vis

    return modified_obs
