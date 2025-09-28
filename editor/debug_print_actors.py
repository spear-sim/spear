#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear
import spear.utils.editor_utils
import unreal


unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)

editor_world_name = unreal_editor_subsystem.get_editor_world().get_name()


if __name__ == "__main__":

    spear.log("Processing scene: ", editor_world_name)

    actors = spear.utils.editor_utils.find_actors()
    for actor in actors:
        spear.log(f"    {spear.utils.editor_utils.get_stable_name_for_actor(actor=actor)}")
        components = spear.utils.editor_utils.get_components(actor=actor)
        for component in components:
            spear.log(f"        {spear.utils.editor_utils.get_stable_name_for_component(component=component)}")
