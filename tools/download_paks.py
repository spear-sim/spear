#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import fnmatch
import os
import posixpath
import spear
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
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--platform", required=True)
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--scene_ids")
    args = parser.parse_args()

    assert args.platform in ["Windows", "Linux", "Mac"]

    # create output dir
    output_dir = os.path.realpath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    scene_ids = []
    if args.scene_ids is None:
        scene_ids = available_scene_ids
    else:
        arg_scene_id_strings = args.scene_ids.split(",")
        for arg_scene_id_string in arg_scene_id_strings:
            scene_ids.extend([ available_scene_id for available_scene_id in available_scene_ids if fnmatch.fnmatch(available_scene_id, arg_scene_id_string) ])

    # construct url and download files
    download_url_prefix = "https://d3q9jkhps5jb4b.cloudfront.net"
    for scene_id in scene_ids:

        pak_file_name = scene_id + "-" + args.version_tag + "-" + args.platform + ".pak"
        download_url  = posixpath.join(download_url_prefix, args.version_tag, pak_file_name)
        output_file   = os.path.join(output_dir, pak_file_name)

        spear.log(f"Downloading {download_url} to {output_file}")

        wget.download(download_url, out=output_file)

    spear.log("Done.")
