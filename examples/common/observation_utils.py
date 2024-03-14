#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np

def get_observation_components_modified_for_visualization(observation, observation_components_to_modify):

    modified_observation = {}
    for modification_mode, observation_component_names in observation_components_to_modify.items():
        for observation_component_name in observation_component_names:

            observation_component = observation[observation_component_name].copy()

            if modification_mode in ["depth", "final_color", "normal", "segmentation"]:
                assert len(observation_component.shape) == 3  # width, height, #channels
                assert observation_component.shape[2] == 4    # 4 channels (RGBA)

            if modification_mode == "depth":
                modified_observation_component = observation_component[:,:,[0,1,2]] # depth is returned as RGBA

                # discard very large depth values
                max_depth_meters = 20.0
                modified_observation_component = modified_observation_component[:,:,0]
                modified_observation_component = np.clip(modified_observation_component, 0.0, max_depth_meters)

            elif modification_mode == "final_color":
                modified_observation_component = observation_component[:,:,[2,1,0]] # final_color is returned as BGRA

            elif modification_mode == "normal":
                modified_observation_component = observation_component[:,:,[0,1,2]] # normal is returned as RGBA

                # discard normals that aren't properly normalized, i.e., length of 1.0
                discard_mask = np.logical_not(np.isclose(np.linalg.norm(modified_observation_component, axis=2), 1.0, rtol=0.001, atol=0.001))
                modified_observation_component = np.clip((modified_observation_component + 1.0) / 2.0, 0.0, 1.0)
                modified_observation_component[discard_mask] = np.nan

            elif modification_mode == "segmentation":
                modified_observation_component = observation_component[:,:,[2,1,0]] # segmentation is returned as BGRA

            else:
                assert False
        
            modified_observation[observation_component_name] = modified_observation_component

    return modified_observation
