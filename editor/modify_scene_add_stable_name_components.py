#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import spear
import spear.utils.editor_utils
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--save", action="store_true")
args = parser.parse_args()

level_editor_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
subobject_data_subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)

editor_world_name = unreal_editor_subsystem.get_editor_world().get_name()


if __name__ == "__main__":

    spear.log("Processing scene: ", editor_world_name)

    scene_modified = False

    for actor in spear.utils.editor_utils.find_actors():
        spear.log("    Processing actor: ", spear.utils.editor_utils.get_stable_name_for_actor(actor))

        actor_subobject_data_handles = subobject_data_subsystem.k2_gather_subobject_data_for_instance(actor)
        actor_subobject_descs = spear.utils.editor_utils.get_subobject_descs(actor_subobject_data_handles)
        assert len(actor_subobject_descs) >= 1
        assert isinstance(actor_subobject_descs[0]["object"], unreal.Actor)

        actor_has_stable_name_component = False
        for actor_subobject_desc in actor_subobject_descs:
            if isinstance(actor_subobject_desc["object"], unreal.SpStableNameComponent):
                actor_has_stable_name_component = True
                spear.log("        Actor has an unreal.unreal.SpStableNameComponent component, skipping...")
                break

        if not actor_has_stable_name_component:
            spear.log("        Creating unreal.SpStableNameComponent...")

            if len(actor_subobject_descs) == 0:
                assert False
            elif len(actor_subobject_descs) == 1:
                parent_data_handle = actor_subobject_descs[0]["data_handle"] # the 0th entry always refers to the actor itself
            else:
                parent_data_handle = actor_subobject_descs[1]["data_handle"] # the 1st entry always refers to the actor's root component

            sp_stable_name_component_desc = spear.utils.editor_utils.add_new_subobject_to_actor(
                actor=actor,
                parent_data_handle=parent_data_handle,
                subobject_name="sp_stable_name_component",
                subobject_class=unreal.SpStableNameComponent)

            scene_modified = False

    if scene_modified and args.save:
        spear.log("Saving scene: ", editor_world_name)
        success = level_editor_subsystem.save_current_level()
        assert success
