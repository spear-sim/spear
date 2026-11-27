#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import os
import spear
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--content-dir", required=True)
parser.add_argument("--export-dir", required=True)
args = parser.parse_args()

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()


def process_content():

    spear.log("Processing content dir: " + args.content_dir)

    asset_datas = asset_registry.get_assets_by_path(package_path=args.content_dir, recursive=True, include_only_on_disk_assets=False)
    asset_datas = sorted(asset_datas, key=lambda ad: f"{ad.package_name}.{ad.asset_name}")
    assets = { f"{ad.package_name}.{ad.asset_name}": spear.editor.get_asset_desc(asset_data=ad) for ad in asset_datas }

    unreal_metadata_dir = spear.path.realpath(args.export_dir, "unreal_metadata")
    content_json_file = spear.path.realpath(unreal_metadata_dir, "content.json")
    spear.log("Writing JSON file: " + content_json_file)
    os.makedirs(unreal_metadata_dir, exist_ok=True)
    with open(content_json_file, "w") as f:
        json.dump(assets, f, indent=4, sort_keys=True)

    spear.log("Done.")


if __name__ == "__main__":
    process_content()
