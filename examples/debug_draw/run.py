#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import colorsys
import numpy as np
import os
import spear


np.random.seed(0)


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    with instance.begin_frame():

        kismet_system_library = game.get_unreal_object(uclass="UKismetSystemLibrary")
        actors = game.unreal_service.find_actors()

        for actor in actors:
            static_mesh_components = game.unreal_service.get_components_by_class(actor=actor, uclass="UStaticMeshComponent")

            for static_mesh_component in static_mesh_components:
                local_bounds = static_mesh_component.GetLocalBounds(Min={"X": 0.0, "Y": 0.0, "Z": 0.0}, Max={"X": 0.0, "Y": 0.0, "Z": 0.0}, as_dict=True)
                local_min = spear.to_numpy_array_from_vector(local_bounds["Min"], as_matrix=True)
                local_max = spear.to_numpy_array_from_vector(local_bounds["Max"], as_matrix=True)
                local_center = (local_min + local_max)/2.0
                local_extent = (local_max - local_min)/2.0

                component_location = spear.to_numpy_array_from_vector(static_mesh_component.K2_GetComponentLocation(), as_matrix=True)
                component_rotation = spear.to_numpy_matrix_from_rotator(static_mesh_component.K2_GetComponentRotation(), as_matrix=True)
                component_scale = np.matrix(np.diag(spear.to_numpy_array_from_vector(static_mesh_component.K2_GetComponentScale())))

                world_center = component_location + component_rotation*component_scale*local_center
                world_extent = np.abs(component_scale*local_extent)

                rgb = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)
                kismet_system_library.DrawDebugBox(
                    Center=spear.to_vector_from_numpy_array(world_center),
                    Extent=spear.to_vector_from_numpy_array(world_extent),
                    LineColor={"R": rgb[0], "G": rgb[1], "B": rgb[2], "A": 1.0},
                    Rotation=spear.to_rotator_from_numpy_matrix(component_rotation),
                    Duration=60.0,
                    Thickness=2.0)

    with instance.end_frame():
        pass

    spear.log("Done.")
