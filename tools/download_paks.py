#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import fnmatch
import os
import posixpath
import spear
from tqdm import tqdm
import urllib.request

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--platform", required=True)
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--scene_ids", required=True)
    args = parser.parse_args()

    assert args.platform in ["Windows", "Linux", "Mac"]

    # create output dir
    output_dir = os.path.realpath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    # create a list of available scene_ids
    available_scene_ids = [ f"kujiale_{str(i).zfill(4)}" for i in range(30) ]
    available_scene_ids.append("warehouse_0000")

    arg_scene_id_strings = args.scene_ids.split(",")
    scene_ids = []
    for arg_scene_id_string in arg_scene_id_strings:
        scene_ids.extend([ available_scene_id for available_scene_id in available_scene_ids if fnmatch.fnmatch(available_scene_id, arg_scene_id_string) ])

    # construct url and download files
    download_url_prefix = "https://d3q9jkhps5jb4b.cloudfront.net"
    block_size = 1024
    for scene_id in scene_ids:

        pak_file_name = scene_id + "-" + args.version_tag + "-" + args.platform + ".pak"
        download_url  = posixpath.join(download_url_prefix, args.version_tag, pak_file_name)
        output_file   = os.path.join(output_dir, pak_file_name)

        spear.log(f"Downloading {download_url} to {output_file}")

        # create a progress bar
        response = urllib.request.urlopen(download_url)
        total_size = int(response.info().get('Content-Length').strip())
        tqdm_bar = tqdm(total=total_size, unit='iB', unit_scale=True)

        with open(output_file, 'wb') as file:
            while True:
                buffer = response.read(block_size)
                if not buffer:
                    break

                file.write(buffer)
                tqdm_bar.update(len(buffer))
        tqdm_bar.close()

    spear.log("Done.")
