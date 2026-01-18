#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import spear
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--load-level", action="store_true")
parser.add_argument("--save-level", action="store_true")
args = parser.parse_args()

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
editor_actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
level_editor_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)


if __name__ == "__main__":

    # Explicitly load "/SpContent" into the asset registry, since it won't be loaded by default if we are
    # running as a commandlet, i.e., when the editor is invoked from the command-line with "-run=pythonscript"
    # instead of "-executepythonscript".
    asset_registry.scan_paths_synchronous(paths=["/SpContent"])

    # Explicitly load level, which is required if we're running as a commandlet.
    if args.load_level:
        level_editor_subsystem.load_level(asset_path="/Game/SPEAR/Scenes/apartment_0000/Maps/apartment_0000")

    # spawn object
    bp_axes_class = unreal.EditorAssetLibrary.load_blueprint_class(asset_path="/SpContent/Blueprints/BP_Axes")
    bp_axes = unreal.EditorLevelLibrary.spawn_actor_from_class(actor_class=bp_axes_class, location=unreal.Vector(x=-10.0, y=280.0, z=50.0))
    spear.log("bp_axes: ", bp_axes)

    # get scale
    scale_3d = bp_axes.get_actor_scale3d()
    spear.log("scale_3d: ", scale_3d.get_editor_property("x"), ", ", scale_3d.get_editor_property("y"), ", ", scale_3d.get_editor_property("z"))

    # set scale and get it again to verify that it has been updated
    bp_axes.set_actor_scale3d(unreal.Vector(x=4.0, y=4.0, z=4.0))
    scale_3d = bp_axes.get_actor_scale3d()
    spear.log("scale_3d: ", scale_3d.get_editor_property("x"), ", ", scale_3d.get_editor_property("y"), ", ", scale_3d.get_editor_property("z"))

    # Explicitly save changes to level.
    if args.save_level:
        level_editor_subsystem.save_current_level()

    spear.log("Done.")
