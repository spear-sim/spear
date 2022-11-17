import getopt
import json
import os
import sys
import urllib.request

CDN_API = "https://kloudsim-usa-cos.kujiale.com/Samples_i/dataset-repo/"

SCENE_DATA_FOLDER = os.path.join(os.path.dirname(__file__), "download_data")
TEMP_FOLDER = os.path.join(SCENE_DATA_FOLDER, "Temp")
NUM_RETRIES = 8


def create_folder(folder_path):
    try:
        if not os.path.exists(folder_path):
            os.makedirs(folder_path, exist_ok=True)
    except:
        pass


# create folder
create_folder(SCENE_DATA_FOLDER)
create_folder(TEMP_FOLDER)


def download_file_from_url(url, local_path, num_retry=NUM_RETRIES):
    try:
        urllib.request.urlretrieve(url, local_path)
    except:
        if num_retry > 0:
            download_file_from_url(url, local_path, num_retry - 1)
    if os.path.exists(local_path):
        return True
    else:
        return False


class SceneDownloader():
    def __init__(self, vw_id, version):
        self.vw_id = vw_id
        self.version = version
        self.vw_info = {}
        self.vw_data = {}

    # download single file
    def download_file_by_path(self, remote_path, output_directory="", is_temp=True, is_force_update=False):

        remote_url = f"{CDN_API}{self.version}/{remote_path}"
        # use user specified directory for target files
        folder_path = (TEMP_FOLDER if is_temp else (output_directory if output_directory != "" else SCENE_DATA_FOLDER))

        local_path = os.path.abspath(os.path.join(folder_path, self.version, self.vw_id, remote_path))
        local_dir, fname = os.path.split(local_path)
        create_folder(local_dir)
        print(remote_url, local_path)
        if not is_force_update and os.path.exists(local_path):
            return local_path
        if download_file_from_url(remote_url, local_path):
            return local_path
        else:
            return None

    # access the info.json to find what contents are available for download
    def load_info(self):
        info_path_local = self.download_file_by_path(f"info/{self.vw_id}_info.json", is_temp=True)
        self.vw_info = json.load(open(info_path_local, mode="r"))

    # access to data.json which contains detail scene description
    def load_data(self):
        data_path_local = self.download_file_by_path(f"data/{self.vw_id}_data.json", is_temp=True)
        self.vw_data = json.load(open(data_path_local, mode="r"))

    # download process
    def process(self, content_type, output_directory, force_update):
        self.load_info()
        asset_mode = self.vw_info["default_asset_mode"]

        if content_type is None or content_type == "":
            pass
        elif content_type in self.vw_info:
            asset_mode = content_type
        else:
            print(f"invalid content type {content_type} for {self.vw_id}")
            return False

        if asset_mode.startswith("pak_split_"):
            assert asset_mode[0:10] == "pak_split_"
            self.download_pak(asset_mode[10:], output_directory, force_update)
        else:
            assets_relative_path = self.vw_info[asset_mode]
            result_local_path = []
            for asset_relative_path in assets_relative_path:
                asset_local_path = self.download_file_by_path(asset_relative_path,
                                                              output_directory=output_directory,
                                                              is_temp=False,
                                                              is_force_update=force_update, )
                if asset_local_path is None:
                    print(f"download asset failed for {self.vw_id}: {asset_relative_path}")
                else:
                    result_local_path.append(asset_local_path)
            print(f"download complete for {self.vw_id}: {result_local_path}")

    def download_pak(self, platform, output_directory, force_update):
        if platform not in ['linux', 'windows', 'mac']:
            return False
        self.load_data()
        download_paths = set()
        # find required resources
        umap = f'paks/{platform}/map/{self.vw_id}.pak'
        download_paths.add(umap)
        for obj in self.vw_data['models']:
            mesh_name = obj['meshName']
            mesh_path = f'paks/{platform}/furniture/{mesh_name}.pak'
            download_paths.add(mesh_path)
            for component in obj['components']:
                for material in obj['components'][component]['materials']:
                    materialName = material['materialName']
                    material_path = f'paks/{platform}/material/{materialName}.pak'
                    download_paths.add(material_path)
        # download
        for remote_path in download_paths:
            self.download_file_by_path(remote_path, output_directory, is_temp=False, is_force_update=force_update)
        return True


def print_help():
    print(
        """scene_downloader.py -i <option> -v <necessary> -f <option> -c <content type> -o <output_dir>
 -i: Specify to download a VirtualWorld. if not use -i, the script will load all virtualworld-ids in data/virtualworld-ids.json.
 -v: it shoud be v1 v2 or v{n}. The newly version information in dataset-repo-update.log.
 -d: if you want to donwload DerivedDataCache, set '-d true'. 
 -f: if '-f true', when downloading, the existing assets will be overwritten. if not use -f, comparing local version information(MD5 in it) to remote version information and decide whether to download asset.
 -o: content saved directory, content will be saved at `<output_dir>/<version>/<relative_file_path>`. if not specified, content will be saved in Saved/<version>/<relative_file_path>
 -c: optional. Download content type for supplementary data. If not specified, download scene data. Valid content types: `topview`, `topview_semantic`,`panorama`.
"""
    )
    sys.exit(2)


if __name__ == "__main__":
    virtualworld_id = ""
    version = ""
    content_type = ''
    output_directory = ""
    force_update = False

    try:
        opts, args = getopt.getopt(
            sys.argv[1:],
            "h:i:v:d:f:p:o:c:",
            [
                "help=",
                "infile=",
                "version=",
                "downDDC=",
                "forceUpdate=",
                "proxy=",
                "outputDir=",
                "contentType=",
            ],
        )
    except getopt.GetoptError:
        print_help()

    for opt, arg in opts:
        if opt == ("-h", "--help"):
            print_help()
        elif opt in ("-i", "--infile"):
            virtualworld_id = arg
        elif opt in ("-v", "--version"):
            version = arg
        elif opt in ("-o", "--outputDir"):
            output_directory = str(arg)
        elif opt in ("-c", "--contentType"):
            content_type = str(arg)
        elif opt in ("-f", "--force"):
            force_update = True

    if version == "":
        print("Error: version is required. Please refer to data/dataset-repo-update.md")
        print_help()

    if virtualworld_id == "":
        # load virtualworld ids from file
        virtualworld_ids_file = os.path.join(os.path.dirname(__file__), "data/virtualworld-ids.json")
        if os.path.exists(virtualworld_ids_file):
            virtualworld_id_list = json.load(open(virtualworld_ids_file))
            for virtual_world_id in virtualworld_id_list:
                sceneDownloader = SceneDownloader(virtual_world_id, version)
                sceneDownloader.process(content_type, output_directory, force_update)
    else:
        # download specific scene
        sceneDownloader = SceneDownloader(virtualworld_id, version)
        sceneDownloader.process(content_type, output_directory, force_update)
