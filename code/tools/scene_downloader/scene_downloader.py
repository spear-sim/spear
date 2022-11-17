import argparse
import json
import os
import pandas as pd
import urllib.request


CDN_PATH = "https://kloudsim-usa-cos.kujiale.com/Samples_i/dataset-repo"


def download_file(source, destination, args):

    destination = os.path.abspath(destination)

    if args.skip_download_if_exists and os.path.exists(destination):
        print(destination + " exists, skipping...")
        return

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
                assert False

    assert os.path.exists(destination)


def download_files(file_descs, args):
    for file_desc in file_descs:
        download_file(file_desc["source"], file_desc["destination"], args)


def download_scene_pak_files(scene_id, args):

    print("Downloading PAK files for scene " + scene_id + " into " + args.destination_dir)

    # download scene metadata
    scene_metadata_json_remote_path = os.path.join(CDN_PATH, args.version, "data", scene_id + "_data.json")
    scene_metadata_json_temp_path   = os.path.join(args.temp_dir, "data", scene_id + "_data.json")
    download_file(scene_metadata_json_remote_path, scene_metadata_json_temp_path, args)
    scene_metadata = json.load(open(scene_metadata_json_temp_path, mode="r"))

    # build a list of pak files to download
    file_descs = []

    # map file
    file_desc = {
        "source":      os.path.join(CDN_PATH, args.version, "paks", args.platform, "map", scene_id + ".pak"),
        "destination": os.path.join(args.destination_dir, args.version, args.platform, scene_id, "map", scene_id + ".pak")}
    if file_desc not in file_descs:
        file_descs.append(file_desc)

    for model in scene_metadata["models"]: # scene_data["models"] is a list, so we iterate over items

        # mesh files
        mesh_id = model["meshName"]
        file_desc = {
            "source":      os.path.join(CDN_PATH, args.version, "paks", args.platform, "furniture", mesh_id + ".pak"),
            "destination": os.path.join(args.destination_dir, args.version, args.platform, scene_id, "furniture", mesh_id + ".pak")}
        if file_desc not in file_descs:
            file_descs.append(file_desc)

        for component_id in model["components"]: # model["components"] is a dict, so we iterate over keys

            component = model["components"][component_id]

            for material in component["materials"]: # component["materials"] is a list, so we iterate over items

                # material files
                material_id = material["materialName"]
                file_desc = {
                    "source":      os.path.join(CDN_PATH, args.version, "paks", args.platform, "material", material_id + ".pak"),
                    "destination": os.path.join(args.destination_dir, args.version, args.platform, scene_id, "material", material_id + ".pak")}
                if file_desc not in file_descs:
                    file_descs.append(file_desc)

    download_files(file_descs, args)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--destination_dir", required=True)
    parser.add_argument("--platform", required=True)
    parser.add_argument("--temp_dir", default="tmp")
    parser.add_argument("--version", default="v7")
    parser.add_argument("--scene_id", default="")
    parser.add_argument("--num_retries", type=int, default=1)
    parser.add_argument("--skip_download_if_exists", default=True)
    parser.add_argument("--proxy")
    args = parser.parse_args()

    # if the user provies a proxy, install it
    if args.proxy_host != "":
        proxy_handler = urllib.request.ProxyHandler({"https": args.proxy, "http": args.proxy})
        opener = urllib.request.build_opener(proxy_handler)
        urllib.request.install_opener(opener=opener)
        urllib.request.urlretrieve(url, local_path)

    # if user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id == "":
        scenes_csv_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "scenes.csv")
        print(scenes_csv_file)
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file, dtype={"scene_id":str})["scene_id"]
    else:
        scene_ids = [args.scene_id]

    # download scenes
    for scene_id in scene_ids:
        download_scene_pak_files(scene_id, args)
