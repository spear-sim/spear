#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import fnmatch
import os
import posixpath
import spear
import sys
import wget


# list all available scene_ids for download
available_scene_ids = [
    "common",
    "kujiale_0000",
    "kujiale_0001",
    "kujiale_0002",
    "kujiale_0003",
    "kujiale_0004",
    # "kujiale_0005",
    "kujiale_0006",
    "kujiale_0007",
    "kujiale_0008",
    "kujiale_0009",
    # "kujiale_0010",
    "kujiale_0011",
    "kujiale_0012",
    "kujiale_0013",
    "kujiale_0014",
    "kujiale_0015",
    "kujiale_0016",
    "kujiale_0017",
    "kujiale_0018",
    "kujiale_0019",
    "kujiale_0020",
    "kujiale_0021",
    "kujiale_0022",
    "kujiale_0023",
    # "kujiale_0024",
    "kujiale_0025",
    # "kujiale_0026",
    "kujiale_0027",
    # "kujiale_0028",
    "kujiale_0029",
    "kujiale_0030",
    "kujiale_0031",
    "kujiale_0032",
    "kujiale_0033",
    "warehouse_0000"]


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--paks_dir", required=True)
    parser.add_argument("--scene_ids")
    parser.add_argument("--version_tag")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    if sys.platform == "win32":
        platform = "Windows"
    elif sys.platform == "darwin":
        platform = "Mac"
    elif sys.platform == "linux":
        platform = "Linux"
    else:
        assert False

    scene_ids = []
    if args.scene_ids is None:
        scene_ids = available_scene_ids
    else:
        arg_scene_id_strings = args.scene_ids.split(",")
        for arg_scene_id_string in arg_scene_id_strings:
            scene_ids.extend([ available_scene_id for available_scene_id in available_scene_ids if fnmatch.fnmatch(available_scene_id, arg_scene_id_string) ])

    if args.version_tag is not None:
        version_tag = args.version_tag
    else:
        version_tag = spear.__version__

    # create output dir
    paks_version_dir = os.path.realpath(os.path.join(args.paks_dir, version_tag))
    os.makedirs(paks_version_dir, exist_ok=True)

    # construct url and download files
    pak_url_prefix = "https://d3q9jkhps5jb4b.cloudfront.net"
    for scene_id in scene_ids:

        pak_file = scene_id + "-" + version_tag + "-" + platform + ".pak"
        pak_url  = posixpath.join(pak_url_prefix, version_tag, pak_file)
        pak_path = os.path.join(paks_version_dir, pak_file)

        if os.path.exists(pak_path) and args.overwrite:
            spear.log(f"File exists, removing: {pak_path}")
            os.remove(pak_path)

        if not os.path.exists(pak_path):
            spear.log(f"Downloading {pak_url} to {pak_path}")
            wget.download(pak_url, out=pak_path)
            print() # wget doesn't seem to print a newline character when it is finished downloading
        else:
            spear.log(f"File exists, skipping: {pak_path}")

    spear.log("Done.")
