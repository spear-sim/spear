#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear
import unreal


asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
editor_actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
level_editor_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)

if __name__ == "__main__":

    # Explicitly load "/SpComponents" into the asset registry, since it won't be loaded by default if we are
    # running as a commandlet, i.e., when the editor is invoked from the command-line with -run=pythonscript
    # as opposed to -ExecutePythonScript.
    asset_registry.scan_paths_synchronous(["/SpComponents"])

    # Explicitly load level, which is required if we're running as a commandlet.
    # level_editor_subsystem.load_level("/Game/Spear/Scenes/apartment_0000/Maps/apartment_0000")

    # spawn object
    bp_axes_uclass = unreal.load_object(outer=None, name="/SpComponents/Blueprints/BP_Axes.BP_Axes_C")
    bp_axes = unreal.EditorLevelLibrary.spawn_actor_from_class(actor_class=bp_axes_uclass, location=unreal.Vector(-10.0, 280.0, 50.0))
    spear.log("bp_axes: ")

    # get scale
    scale_3d = bp_axes.get_actor_scale3d()
    spear.log("scale_3d: ", scale_3d.get_editor_property("x"), ", ", scale_3d.get_editor_property("y"), ", ", scale_3d.get_editor_property("z"))

    # set scale and get it again to verify that it updated
    bp_axes.set_actor_scale3d(unreal.Vector(4.0, 4.0, 4.0))
    scale_3d = bp_axes.get_actor_scale3d()
    spear.log("scale_3d: ", scale_3d.get_editor_property("x"), ", ", scale_3d.get_editor_property("y"), ", ", scale_3d.get_editor_property("z"))

    print(level_editor_subsystem.get_current_level())

    # Explicitly save changes to level.
    # level_editor_subsystem.save_current_level()

    spear.log("Done.")
