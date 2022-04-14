import getopt
import json
import os
import sys

import scene_manager


def print_help():
    print(
        """scene_manager_meta.py -v <version> -i <virtual-world-id>  -p <proxy>
e.g: scene_manager_images.py -v v1
 -v: required. VirtualWorld version in format of v{n}, e.g. v1, v2. The latest version information is in /VirtualWrold/SceneManager/dataset-repo-update.log.
 -i: optional. Specify to download images for VirtualWorld. if not use -i, the script will load all virtualworld-ids in /VirtualWrold/SceneManager/Data/virtualworld-ids.json.
 -p: optional. setup proxy
"""
    )
    sys.exit(2)


DEFAULT_SAVE_DIR = "../Saved/images"

IMAGE_FILE_NAMES = ["topview.png", "topview_semantic.png", "topview_data.json"]


def download_images(virtualworld_id, version, save_dir):
    if not os.path.exists(save_dir):
        os.mkdir(save_dir)
    print(f"download_images - {virtualworld_id} {version}")
    for filename in IMAGE_FILE_NAMES:
        file_type = filename.split(".")[0]
        file_extension = filename.split(".")[1]
        metadata_url = f"{scene_manager.CDN_API}scenes/{virtualworld_id}/{version}/{filename}"
        metadata_local = f"{save_dir}/{file_type}_{virtualworld_id}_{version}.{file_extension}"
        if not os.path.exists(metadata_local):
            result = scene_manager.download_file_from_url(metadata_url, metadata_local)


if __name__ == "__main__":
    version = ""
    virtualworld_id = ""
    save_dir = DEFAULT_SAVE_DIR

    try:
        opts, args = getopt.getopt(
            sys.argv[1:], "h:i:v:d:f:p:", ["help=", "infile=", "version=", "proxy="]
        )
    except getopt.GetoptError:
        print_help()

    for opt, arg in opts:
        if opt == ("-h", "--help"):
            print_help()
        elif opt in ("-i", "--infile"):
            virtualworld_id = arg
        elif opt in ("-v", "--version"):
            if "v" in arg:
                version = arg
        elif opt in ("-o", "--out_dir"):
            if os.path.isfile(arg):
                print(f"invalid output_dir: {arg}")
            else:
                save_dir = arg
        elif opt in ("-p", "--proxy"):
            scene_manager.PROXY_URL = str(arg)

    if version == "":
        print(
            "Error: set version correct. please select version from SceneManage/Data/dataset-repo-update.log"
        )
        print_help()

    if virtualworld_id == "":
        virtualworld_ids_file = os.path.join(
            os.path.dirname(__file__), "Data/virtualworld-ids.json"
        )
        if os.path.exists(virtualworld_ids_file):
            with open(virtualworld_ids_file) as f:
                for id in json.load(f):
                    download_images(id, version, save_dir)
    else:
        download_images(virtualworld_id, version, save_dir)
