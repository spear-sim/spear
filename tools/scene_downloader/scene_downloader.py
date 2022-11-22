import argparse
import json
import os
import pandas as pd
import posixpath
import urllib.request


CDN_PATH = "https://kloudsim-usa-cos.kujiale.com/Samples_i/dataset-repo"


def download_file(source, destination, args):

    destination = os.path.abspath(destination)

    if args.force_overwrite or not os.path.exists(destination):
        os.makedirs(os.path.dirname(destination), exist_ok=True)
        for i in range(args.num_retries):
            print("Downloading " + source + "...")
            try:
                urllib.request.urlretrieve(url=source, filename=destination)
                break
            except:
                if i < args.num_retries - 1:
                    print("Error occurred while downloading, retrying...")
                else:
                    print("Error occurred while downloading and the maximum number of retries has been reached. Giving up.")
            if i == args.num_retries -1:
                assert False

    else:
        print(destination + " exists, skipping...")
        return

    assert os.path.exists(destination)


def download_scene_pak_files(scene_id, args):

    print("Downloading PAK files for scene " + scene_id + " into " + args.data_dir)

    # download scene metadata
    scene_metadata_json_remote_path = posixpath.join(CDN_PATH, args.version, "data", scene_id + "_data.json")
    scene_metadata_json_temp_path   = os.path.join(args.temp_dir, "data", scene_id + "_data.json")
    download_file(scene_metadata_json_remote_path, scene_metadata_json_temp_path, args)
    scene_metadata = json.load(open(scene_metadata_json_temp_path, mode="r"))

    # build a list of pak files to download, taking care not to add duplicates
    file_descs = []

    # shared asset file (could be downloaded once for all scenes, but we download for each scene for simplicity)
    file_desc = {
        "source":      posixpath.join(CDN_PATH, args.version, "paks", args.platform, "koolab.pak"),
        "destination": os.path.join(args.data_dir, args.version, args.platform, scene_id, "koolab.pak")}
    if file_desc not in file_descs:
        file_descs.append(file_desc)

    # map file
    file_desc = {
        "source":      posixpath.join(CDN_PATH, args.version, "paks", args.platform, "map", scene_id + ".pak"),
        "destination": os.path.join(args.data_dir, args.version, args.platform, scene_id, "map", scene_id + ".pak")}
    if file_desc not in file_descs:
        file_descs.append(file_desc)

    for model in scene_metadata["models"]: # scene_data["models"] is a list, so we iterate over items

        # mesh files
        mesh_id = model["meshName"]
        file_desc = {
            "source":      posixpath.join(CDN_PATH, args.version, "paks", args.platform, "furniture", mesh_id + ".pak"),
            "destination": os.path.join(args.data_dir, args.version, args.platform, scene_id, "furniture", mesh_id + ".pak")}
        if file_desc not in file_descs:
            file_descs.append(file_desc)

        for component_id in model["components"]: # model["components"] is a dict, so we iterate over keys

            component = model["components"][component_id]

            for material in component["materials"]: # component["materials"] is a list, so we iterate over items

                # material files
                material_id = material["materialName"]
                file_desc = {
                    "source":      posixpath.join(CDN_PATH, args.version, "paks", args.platform, "material", material_id + ".pak"),
                    "destination": os.path.join(args.data_dir, args.version, args.platform, scene_id, "material", material_id + ".pak")}
                if file_desc not in file_descs:
                    file_descs.append(file_desc)

    # download the files
    for file_desc in file_descs:
        download_file(file_desc["source"], file_desc["destination"], args)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", required=True)
    parser.add_argument("--platform", required=True)
    parser.add_argument("--temp_dir", default="tmp")
    parser.add_argument("--version", default="v7")
    parser.add_argument("--num_retries", type=int, default=1)
    parser.add_argument("--scene_id")
    parser.add_argument("--proxy")
    parser.add_argument("--force_overwrite", action="store_true")
    args = parser.parse_args()

    # proceed with only supported platforms
    if args.platform not in ["windows", "mac", "linux"]:
        assert False

    # if the user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id is None:
        scenes_csv_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "scenes.csv")
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file, dtype={"scene_id":str})["scene_id"]
    else:
        scene_ids = [args.scene_id]

    # if the user provides a proxy, install it
    if args.proxy is not None:
        assert args.proxy != ""
        proxy_handler = urllib.request.ProxyHandler({"https": args.proxy, "http": args.proxy})
        opener = urllib.request.build_opener(proxy_handler)
        urllib.request.install_opener(opener=opener)

    # download scenes
    for scene_id in scene_ids:
        download_scene_pak_files(scene_id, args)
