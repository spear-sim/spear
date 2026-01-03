#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import numpy as np
import spear
import spear.utils.editor_utils
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--save", action="store_true")
parser.add_argument("--skip-add-stable-name-component", action="store_true")
args = parser.parse_args()

editor_actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
level_editor_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
subobject_data_subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)

editor_world_name = unreal_editor_subsystem.get_editor_world().get_name()

np.set_printoptions(suppress=True)


if __name__ == "__main__":

    spear.log("Processing scene: ", editor_world_name)

    scene_modified = False

    nav_mesh_bounds_volumes = spear.utils.editor_utils.find_actors(actor_class=unreal.NavMeshBoundsVolume)

    if len(nav_mesh_bounds_volumes) == 0:
        spear.log("    Scene doesn't have an unreal.NavMeshBoundsVolume actor, computing bounds...")

        scene_has_valid_geometry = False
        scene_bounds_min = np.zeros((3))
        scene_bounds_max = np.zeros((3))

        actors = spear.utils.editor_utils.find_actors()
        actors = [ actor for actor in actors if actor.get_editor_property(name="relevant_for_level_bounds") ]

        for actor in actors:
            spear.log("    Processing actor: ", spear.utils.editor_utils.get_stable_name_for_actor(actor))

            components = spear.utils.editor_utils.get_components(actor=actor, component_class=unreal.StaticMeshComponent)

            for component in components:
                scene_has_valid_geometry = True

                origin_ue, half_extent_ue, sphere_radius = unreal.SystemLibrary.get_component_bounds(component=component)
                origin = spear.utils.editor_utils.to_numpy_array_from_vector(vector=origin_ue)
                half_extent = spear.utils.editor_utils.to_numpy_array_from_vector(vector=half_extent_ue)

                spear.log(f"        {spear.utils.editor_utils.get_stable_name_for_component(component=component)} (origin={origin}, half_extent={half_extent}, sphere_radius={sphere_radius})")

                scene_bounds_min = np.minimum(scene_bounds_min, origin - half_extent)
                scene_bounds_max = np.maximum(scene_bounds_max, origin + half_extent)

        if scene_has_valid_geometry:
            spear.log("    Scene has valid scene geometry, spawning unreal.NavMeshBoundsVolume...")

            assert spear.utils.editor_utils.find_actor(stable_name="Navigation/NavMeshBoundsVolume") is None

            scale_units_per_unreal_unit = 1.0/100.0
            nav_mesh_bounds_location = (scene_bounds_min + scene_bounds_max)/2.0
            nav_mesh_bounds_scale = scale_units_per_unreal_unit*(scene_bounds_max - scene_bounds_min)/2.0

            nav_mesh_bounds_volume = editor_actor_subsystem.spawn_actor_from_class(
                actor_class=unreal.NavMeshBoundsVolume,
                location=spear.utils.editor_utils.to_vector_from_numpy_array(array=nav_mesh_bounds_location))
            nav_mesh_bounds_volume.set_folder_path(new_folder_path="Navigation")
            nav_mesh_bounds_volume.set_actor_label(new_actor_label="NavMeshBoundsVolume")
            nav_mesh_bounds_volume.set_actor_scale3d(new_scale3d=nav_mesh_bounds_scale)

            if not args.skip_add_stable_name_component:
                spear.log("    Adding unreal.SpStableNameComponent...")

                nav_mesh_bounds_volume_subobject_descs = spear.utils.editor_utils.get_subobject_descs_for_instance(nav_mesh_bounds_volume)
                assert len(nav_mesh_bounds_volume_subobject_descs) == 2
                assert isinstance(nav_mesh_bounds_volume_subobject_descs[0]["object"], unreal.Actor)          # the 0th entry always refers to the actor itself
                assert isinstance(nav_mesh_bounds_volume_subobject_descs[1]["object"], unreal.BrushComponent) # the 1st entry must be the root component in this case because there are only 2 entries

                parent_data_handle = nav_mesh_bounds_volume_subobject_descs[0]["data_handle"] # actor
                sp_stable_name_component_desc = spear.utils.editor_utils.add_new_subobject_to_instance(
                    parent_data_handle=parent_data_handle,
                    subobject_name="sp_stable_name_component",
                    subobject_class=unreal.SpStableNameComponent)

            scene_modified = True

    else:
        for nav_mesh_bounds_volume in nav_mesh_bounds_volumes:
            origin_ue, half_extent_ue = nav_mesh_bounds_volume.get_actor_bounds(only_colliding_components=False)
            origin = spear.utils.editor_utils.to_array_from_vector(vector=origin_ue)
            half_extent = spear.utils.editor_utils.to_array_from_vector(vector=half_extent_ue)
            spear.log(f"    Scene already has an unreal.NavMeshBoundsVolume actor: {spear.utils.editor_utils.get_stable_name_for_actor(actor=nav_mesh_bounds_volume)} (origin={origin}, half_extent={half_extent})")

    if scene_modified and args.save:
        spear.log("Saving scene: ", editor_world_name)
        success = level_editor_subsystem.save_current_level()
        assert success
