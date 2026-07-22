#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import os
import pandas as pd
import spear
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--content-dir", required=True)
parser.add_argument("--export-dir", required=True)
args = parser.parse_args()

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
dependency_options = unreal.AssetRegistryDependencyOptions(include_soft_package_references=True, include_hard_package_references=True)

editor_properties_csv_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "editor_properties.csv"))
df_editor_properties = pd.read_csv(editor_properties_csv_file, comment="#")


def process_content():

    spear.log("Processing content dir: " + args.content_dir)

    asset_datas = asset_registry.get_assets_by_path(package_path=args.content_dir, recursive=True, include_only_on_disk_assets=False)
    asset_datas = sorted(asset_datas, key=lambda ad: f"{ad.package_name}.{ad.asset_name}")
    assets = { f"{ad.package_name}.{ad.asset_name}": get_asset_desc(asset_data=ad) for ad in asset_datas }

    unreal_metadata_dir = os.path.realpath(os.path.join(args.export_dir, "unreal_metadata"))
    content_json_file = os.path.realpath(os.path.join(unreal_metadata_dir, "content.json"))
    spear.log("Writing JSON file: " + content_json_file)
    os.makedirs(unreal_metadata_dir, exist_ok=True)
    with open(content_json_file, "w") as f:
        json.dump(assets, f, indent=4, sort_keys=True)

    spear.log("Done.")

def get_asset_desc(asset_data):

    asset_path = f"{asset_data.package_name}.{asset_data.asset_name}"
    spear.log("Processing asset: " + asset_path)

    # Immediate (non-recursive) dependencies come from the Asset Registry's cached metadata, so we get the
    # dependency package names without loading the referenced assets.
    dependencies = sorted( str(d) for d in asset_registry.get_dependencies(package_name=asset_data.package_name, dependency_options=dependency_options) )

    asset = asset_data.get_asset()
    return {
        "class": str(asset_data.asset_class_path.asset_name),
        "dependencies": dependencies,
        "editor_properties": get_object_descs(uobject=asset) if asset is not None else None,
        "name": str(asset_data.asset_name),
        "package": str(asset_data.package_name),
        "path": asset_path}

def get_object_descs(uobject):

    # The Unreal Python interface does not provide a mechanism to iterate over editor properties directly,
    # so (as in export_scene.py) we treat each Python attribute as a candidate editor property, augment the
    # candidates with any names listed for this class in editor_properties.csv (to catch properties that are
    # exposed to get_editor_property but not to dir()), and keep the ones get_editor_property() accepts.
    uobject_class_names = [ c.__name__ for c in uobject.__class__.mro() ] # derived-to-base order
    editor_property_candidate_names = set(dir(uobject))
    for uobject_class_name in uobject_class_names:
        editor_property_candidate_names = editor_property_candidate_names | set(df_editor_properties.loc[df_editor_properties["class"] == uobject_class_name]["editor_property"])

    editor_property_descs = {}
    for editor_property_candidate_name in editor_property_candidate_names:
        try:
            editor_property = uobject.get_editor_property(name=editor_property_candidate_name)
            editor_property_descs[editor_property_candidate_name] = get_object_desc(value=editor_property)
        except:
            pass

    return editor_property_descs

def get_object_desc(value):

    # Content-wide dump: emit object references (class + path) rather than recursing into them, since every
    # referenced asset is already its own top-level entry here (and following references could cycle). Structs
    # are inline values, so we do recurse into those.

    if value is None:
        return None
    elif isinstance(value, (unreal.Array, unreal.Set, tuple)):
        return [ get_object_desc(value=entry) for entry in value ]
    elif isinstance(value, unreal.Map):
        return { str(k): get_object_desc(value=v) for k, v in value.items() }
    elif isinstance(value, unreal.StructBase):
        return {"class": value.__class__.__name__, "editor_properties": get_object_descs(uobject=value)}
    elif isinstance(value, unreal.Object):
        return {"class": value.__class__.__name__, "path": value.get_path_name()}
    else:
        try:
            json.dumps(value)
            return value
        except:
            return str(value)


if __name__ == "__main__":
    process_content()
