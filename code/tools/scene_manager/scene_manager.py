import os
import sys
import getopt
import json
import urllib.request
import shutil
from concurrent import futures
from tqdm import tqdm

CDN_API = "https://kloudsim-usa-cos.kujiale.com/Samples_i/dataset-repo/"

SCENE_DATA_FOLDER = os.path.join(os.path.dirname(__file__), "download_data")
TEMP_FOLDER = os.path.join(SCENE_DATA_FOLDER, "Temp")
VERSION_INFO_FOLDER = os.path.join(SCENE_DATA_FOLDER, "version_info")
PROXY_URL = ""

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
create_folder(VERSION_INFO_FOLDER)


def check_update_version_info(local_version_info, remote_version_info):
    if os.path.exists(local_version_info) and os.path.exists(remote_version_info):
        with open(local_version_info) as f:
            local_data = json.load(f)
        with open(remote_version_info) as f:
            remote_data = json.load(f)
        local_md5 = local_data.get("MD5")
        remote_md5 = remote_data.get("MD5")
        if local_md5 is None:
            return True
        if remote_md5 is None:
            return True
        if local_md5 == remote_md5:
            return False
    return True


def check_updade_version(local_version_info, version):
    if os.path.exists(local_version_info):
        with open(local_version_info) as f:
            local_data = json.load(f)
            if not local_data["Version"] == version:
                return True
            else:
                return False
    return True


def download_file_from_url(url, local_path, num_retry=NUM_RETRIES):
    try:
        if PROXY_URL == "":
            urllib.request.urlretrieve(url, local_path)
        else:
            proxy_string = "proxy-chain.intel.com:911"
            proxy = urllib.request.ProxyHandler(
                {"https": proxy_string, "http": proxy_string}
            )
            opener = urllib.request.build_opener(proxy)
            urllib.request.install_opener(opener=opener)
            urllib.request.urlretrieve(url, local_path)
    except:
        if num_retry > 0:
            download_file_from_url(url, local_path, num_retry - 1)
    if os.path.exists(local_path):
        return True
    else:
        return False


def get_scene_config(virtualworld_id, version):
    scene_meta_url = (
            CDN_API
            + "scenes/"
            + virtualworld_id
            + "/"
            + version
            + "/ConfigMeta_{}.json".format(virtualworld_id)
    )
    scene_meta_local = os.path.join(
        TEMP_FOLDER, virtualworld_id, "ConfigMeta_{}.json".format(virtualworld_id)
    )
    create_folder(os.path.join(TEMP_FOLDER, virtualworld_id))
    if download_file_from_url(scene_meta_url, scene_meta_local):
        with open(scene_meta_local) as f:
            return json.load(f)
    return None


def deal_scene_file(remote_url, local_path, dst_path):
    download_file_from_url(remote_url, local_path)
    if os.path.exists(local_path):
        if os.path.exists(dst_path):
            shutil.rmtree(dst_path)
        shutil.unpack_archive(local_path, dst_path)
        if os.path.isdir(dst_path):
            return True
    return False


def download_scenes(virtualworld_id, version, scene_config_data, is_force_update):
    local_version_info_folder = os.path.join(VERSION_INFO_FOLDER, "scenes")
    create_folder(local_version_info_folder)
    version_scene_info_name = "version_info_scene_{}.json".format(virtualworld_id)
    local_version_info = os.path.join(
        local_version_info_folder, version_scene_info_name
    )
    remote_version_info_url = (
            CDN_API
            + "scenes/"
            + virtualworld_id
            + "/"
            + version
            + "/"
            + version_scene_info_name
    )
    remote_version_info = os.path.join(TEMP_FOLDER, version_scene_info_name)
    download_file_from_url(remote_version_info_url, remote_version_info)

    if not is_force_update:
        if not check_update_version_info(local_version_info, remote_version_info):
            return True

    anim_url = (
            CDN_API
            + "scenes/"
            + virtualworld_id
            + "/"
            + version
            + "/"
            + scene_config_data["animation"]
    )
    anim_local = os.path.join(TEMP_FOLDER, scene_config_data["animation"])
    anim_dst = os.path.join(
        os.path.dirname(__file__),
        "../../unreal_projects/RobotProject/Content/Scene/",
        scene_config_data["animation"].replace(".zip", ""),
    )
    is_anim_ok = deal_scene_file(anim_url, anim_local, anim_dst)

    arch_url = (
            CDN_API
            + "scenes/"
            + virtualworld_id
            + "/"
            + version
            + "/"
            + scene_config_data["architecture"]
    )
    arch_local = os.path.join(TEMP_FOLDER, scene_config_data["architecture"])
    arch_dst = os.path.join(
        os.path.dirname(__file__),
        "../../unreal_projects/RobotProject/Content/Scene/Meshes",
        scene_config_data["architecture"].replace(".zip", ""),
    )
    is_arch_ok = deal_scene_file(arch_url, arch_local, arch_dst)

    mat_url = (
            CDN_API
            + "scenes/"
            + virtualworld_id
            + "/"
            + version
            + "/"
            + scene_config_data["materialinst"]
    )
    mat_local = os.path.join(TEMP_FOLDER, scene_config_data["materialinst"])
    mat_dst = os.path.join(
        os.path.dirname(__file__),
        "../../unreal_projects/RobotProject/Content/Scene/",
        scene_config_data["materialinst"].replace(".zip", ""),
    )
    is_mat_ok = deal_scene_file(mat_url, mat_local, mat_dst)

    map_folder = os.path.join(os.path.dirname(__file__), "../../unreal_projects/RobotProject/Content/Maps")
    create_folder(map_folder)
    umap_local = os.path.join(map_folder, scene_config_data["map"])
    download_file_from_url(
        CDN_API
        + "scenes/"
        + virtualworld_id
        + "/"
        + version
        + "/"
        + scene_config_data["map"],
        umap_local,
    )
    is_umap_ok = False
    if os.path.exists(umap_local):
        is_umap_ok = True

    if scene_config_data.get("BuildData") is not None:
        build_data_local = os.path.join(map_folder, scene_config_data["BuildData"])
        download_file_from_url(
            CDN_API
            + "scenes/"
            + virtualworld_id
            + "/"
            + version
            + "/"
            + scene_config_data["BuildData"],
            build_data_local,
        )

        if not os.path.exists(build_data_local):
            return False

    # move versioninfo file
    if is_umap_ok and is_anim_ok and is_arch_ok and is_mat_ok:
        if os.path.exists(local_version_info):
            os.remove(local_version_info)
        shutil.move(remote_version_info, local_version_info)
        # remove temp file
        if os.path.exists(anim_local):
            os.remove(anim_local)
        if os.path.exists(arch_local):
            os.remove(arch_local)
        if os.path.exists(mat_local):
            os.remove(mat_local)
        return True

    return False


def deal_asset_file(content):
    try:
        asset_url = content.get("asset_url")
        asset_local = content.get("asset_local")
        asset_dst = content.get("asset_dst")
        version_info_url = content.get("version_info_url")
        version_info_local = content.get("version_info_local")
        version_info_dst = content.get("version_info_dst")
        download_file_from_url(version_info_url, version_info_local)
        if os.path.exists(asset_dst):
            if not check_update_version_info(version_info_dst, version_info_local):
                if os.path.exists(version_info_local):
                    os.remove(version_info_local)
                return True
        if os.path.exists(asset_local):
            os.remove(asset_local)
        download_file_from_url(asset_url, asset_local)
        if os.path.exists(asset_local):
            if ".zip" in asset_local:
                if os.path.exists(asset_dst):
                    shutil.rmtree(asset_dst)
                shutil.unpack_archive(asset_local, asset_dst)
                if os.path.isdir(asset_dst):
                    if os.path.exists(version_info_dst):
                        os.remove(version_info_dst)
                    shutil.move(version_info_local, version_info_dst)
                    if os.path.exists(asset_local):
                        os.remove(asset_local)
                    return True
            else:
                if os.path.exists(version_info_dst):
                    os.remove(version_info_dst)
                shutil.move(version_info_local, version_info_dst)
                return True
    except:
        pass
    return False


def mult_down(contents):
    is_ok = True
    is_appear = False
    with futures.ThreadPoolExecutor() as pool:
        for result in tqdm(pool.map(deal_asset_file, contents), total=len(contents)):
            if not result and not is_appear:
                is_appear = True
                is_ok = False
    return is_ok


def download_assets(
        virtualworld_id, version, is_down_ddc, scene_config_data, is_force_update
):
    local_version_info_folder = os.path.join(VERSION_INFO_FOLDER, "assets")
    mat_version_info_folder = os.path.join(local_version_info_folder, "materials")
    furniture_version_info_folder = os.path.join(local_version_info_folder, "furniture")
    phys_furniture_version_info_folder = os.path.join(
        local_version_info_folder, "phys_furniture"
    )
    ddc_version_info_folder = os.path.join(local_version_info_folder, "ddc")
    create_folder(mat_version_info_folder)
    create_folder(furniture_version_info_folder)
    create_folder(phys_furniture_version_info_folder)
    if is_down_ddc:
        create_folder(ddc_version_info_folder)

    mat_down_body = []
    for materialid in scene_config_data["material"]:
        dic_curr = {}
        dic_curr["asset_url"] = (
                CDN_API
                + "assets/material/"
                + materialid
                + "/"
                + version
                + "/"
                + materialid
                + ".zip"
        )
        dic_curr["asset_local"] = os.path.join(TEMP_FOLDER, materialid + ".zip")
        dic_curr["asset_dst"] = os.path.join(
            os.path.dirname(__file__), "../../unreal_projects/RobotProject/Content/Scene/Materials", materialid
        )
        dic_curr["version_info_url"] = (
                CDN_API
                + "assets/material/"
                + materialid
                + "/"
                + version
                + "/"
                + "version_info_"
                + materialid
                + ".json"
        )
        dic_curr["version_info_local"] = os.path.join(
            TEMP_FOLDER, "version_info_" + materialid + ".json"
        )
        dic_curr["version_info_dst"] = os.path.join(
            mat_version_info_folder, "version_info_" + materialid + ".json"
        )
        mat_down_body.append(dic_curr)
    print("update {} material".format(virtualworld_id))
    is_mat_ok = mult_down(mat_down_body)

    furniture_down_body = []
    for meshid in scene_config_data["mesh"]:
        dic_curr = {}
        dic_curr["asset_url"] = (
                CDN_API
                + "assets/furniture/"
                + meshid
                + "/"
                + version
                + "/"
                + meshid
                + ".zip"
        )
        dic_curr["asset_local"] = os.path.join(TEMP_FOLDER, meshid + ".zip")
        dic_curr["asset_dst"] = os.path.join(
            os.path.dirname(__file__), "../../unreal_projects/RobotProject/Content/Scene/Meshes/Furniture", meshid
        )
        dic_curr["version_info_url"] = (
                CDN_API
                + "assets/furniture/"
                + meshid
                + "/"
                + version
                + "/"
                + "version_info_"
                + meshid
                + ".json"
        )
        dic_curr["version_info_local"] = os.path.join(
            TEMP_FOLDER, "version_info_" + meshid + ".json"
        )
        dic_curr["version_info_dst"] = os.path.join(
            phys_furniture_version_info_folder, "version_info_" + meshid + ".json"
        )
        furniture_down_body.append(dic_curr)
    print("update {} furniture".format(virtualworld_id))
    is_fur_ok = mult_down(furniture_down_body)

    phys_furniture_down_body = []
    for meshid in scene_config_data["mesh"]:
        dic_curr = {}
        dic_curr["asset_url"] = (
                CDN_API
                + "assets/phys_furniture/"
                + meshid
                + "/"
                + version
                + "/"
                + meshid
                + ".zip"
        )
        dic_curr["asset_local"] = os.path.join(TEMP_FOLDER, meshid + ".zip")
        dic_curr["asset_dst"] = os.path.join(
            os.path.dirname(__file__),
            "../../unreal_projects/RobotProject/Content/Scene/Meshes/PhysicalFurniture",
            meshid,
        )
        dic_curr["version_info_url"] = (
                CDN_API
                + "assets/phys_furniture/"
                + meshid
                + "/"
                + version
                + "/"
                + "version_info_"
                + meshid
                + ".json"
        )
        dic_curr["version_info_local"] = os.path.join(
            TEMP_FOLDER, "version_info_" + meshid + ".json"
        )
        dic_curr["version_info_dst"] = os.path.join(
            furniture_version_info_folder, "version_info_" + meshid + ".json"
        )
        phys_furniture_down_body.append(dic_curr)
    print("update {} phys_furniture".format(virtualworld_id))
    is_phys_fur_ok = mult_down(phys_furniture_down_body)

    if is_down_ddc:
        ddc_down_body = []
        for ddc_path in scene_config_data["DDC"]:
            ddc_only_path = ddc_path.replace(".udd", "")
            ddc_array = ddc_path.split("/")
            ddc_name = ddc_array[len(ddc_array) - 1]
            dic_curr = {}
            dic_curr["asset_url"] = (
                    CDN_API + "assets/ddc/" + ddc_only_path + "/" + version + "/" + ddc_name
            )
            dic_curr["asset_local"] = os.path.join(
                os.path.dirname(__file__), "../../unreal_projects/RobotProject/DerivedDataCache", ddc_path
            )
            if not os.path.exists(os.path.dirname(dic_curr["asset_local"])):
                try:
                    os.makedirs(dic_curr["asset_local"])
                except:
                    pass
            dic_curr["asset_dst"] = os.path.join(
                os.path.dirname(__file__), "../../unreal_projects/RobotProject/DerivedDataCache", ddc_path
            )
            dic_curr["version_info_url"] = (
                    CDN_API
                    + "assets/ddc/"
                    + ddc_only_path
                    + "/"
                    + version
                    + "/"
                    + "version_info_"
                    + ddc_name.replace(".udd", "")
                    + ".json"
            )
            dic_curr["version_info_local"] = os.path.join(
                TEMP_FOLDER, "version_info_" + ddc_name.replace(".udd", "") + ".json"
            )
            dic_curr["version_info_dst"] = os.path.join(
                ddc_version_info_folder,
                "version_info_" + ddc_name.replace(".udd", "") + ".json",
            )
            ddc_down_body.append(dic_curr)
        print("update {} ddc".format(virtualworld_id))
        is_ddc_ok = mult_down(ddc_down_body)
        return is_mat_ok and is_fur_ok and is_phys_fur_ok
    else:
        return is_mat_ok and is_fur_ok and is_phys_fur_ok


def download_single_virtualworld(
        virtualworld_id, version, is_down_ddc, is_force_update
):
    # get scene meta
    scene_config_data = get_scene_config(virtualworld_id, version)
    # download assets
    is_assets_ready = download_assets(
        virtualworld_id, version, is_down_ddc, scene_config_data, is_force_update
    )
    is_scene_ready = False
    # download scenes
    if is_assets_ready:
        is_scene_ready = download_scenes(
            virtualworld_id, version, scene_config_data, is_force_update
        )

    update_log = os.path.join(SCENE_DATA_FOLDER, "UpdateLog")
    if not os.path.exists(update_log):
        os.makedirs(update_log)

    log = os.path.join(update_log, virtualworld_id + "_faild.txt")
    if is_assets_ready and is_scene_ready:
        if os.path.exists(log):
            os.remove(log)
        print("download {} success".format(virtualworld_id))
        return True
    else:
        with open(log, "w") as f:
            f.writelines(
                "Please update {} again. assset is {} and scene is {}".format(
                    virtualworld_id, is_assets_ready, is_scene_ready
                )
            )
        print(
            "Please update {} again. assset is {} and scene is {}".format(
                virtualworld_id, is_assets_ready, is_scene_ready
            )
        )
        return False


# determine process mode based on version
def get_version_process_mode(version):
    if version in ['v1', 'v2', 'v3']:
        return "raw"
    elif version in ['v4']:
        return "pak"
    elif version in ['v5']:
        return "pak"
    else:
        return "unknown"


def download_file_by_path(vw_id, version, path, output_directory="", is_temp=True, is_force_update=False):
    """
        return: local file path
    """
    remote_url = f"{CDN_API}{version}/{path}"
    # use user specified directory for target files
    folder_path = TEMP_FOLDER if is_temp else (
        output_directory if output_directory != '' else SCENE_DATA_FOLDER)
    local_path = os.path.abspath(os.path.join(folder_path, version, vw_id, path))
    local_dir, fname = os.path.split(local_path)
    create_folder(local_dir)
    if not is_force_update and os.path.exists(local_path):
        return local_path
    if download_file_from_url(remote_url, local_path):
        return local_path
    else:
        return None


# download single virtual world
def download_single_virtualworld_pak(vw_id, version, param=None):
    if param is None:
        param = {}

    # download info for basic info and  asset download path
    data_relative_path = f"info/{vw_id}_info.json"
    data_local_path = download_file_by_path(vw_id, version, data_relative_path, is_temp=True)
    if data_local_path is None or data_local_path == "":
        print(f"download data failed {vw_id} {version}")
        return False

    # download asset data
    vw_data = json.load(open(data_local_path, mode='r'))
    asset_mode = vw_data['default_asset_mode']
    if "content_type" in param:
        content_type = param['content_type']
        if content_type in vw_data:
            asset_mode = content_type
        elif content_type is None or content_type == '':
            asset_mode = vw_data['default_asset_mode']
        else:
            print(f"invalid content type {content_type} for {vw_id}")
            return False
    assets_relative_path = vw_data[asset_mode]
    result_local_path = []
    for asset_relative_path in assets_relative_path:
        asset_local_path = download_file_by_path(vw_id, version, asset_relative_path,
                                                 output_directory=param.get("output_directory"), is_temp=False,
                                                 is_force_update=param.get("is_force_update"))
        if asset_local_path is None:
            print(f"download asset failed for {vw_id}: {asset_relative_path}")
        else:
            result_local_path.append(asset_local_path)
    print(f"download complete for {vw_id}: {result_local_path}")
    return True


def print_help():
    print(
        """scene_manager.py -i <option> -v <necessary> -d <option> -f <option>
e.g: scene_manager.py -v v1
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
    version = ""
    virtualworld_id = ""
    param = {
        "output_directory": "",
        "is_down_ddc": False,
        "is_force_update": False,
        "content_type": ''
    }

    try:
        opts, args = getopt.getopt(
            sys.argv[1:],
            "h:i:v:d:f:p:o:c:",
            ["help=", "infile=", "version=", "downDDC=", "forceUpdate=", "proxy=", "outputDir=", "contentType="],
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
        elif opt in ("-f", "--forceUpdate"):
            if arg in ["true", "True"]:
                param["is_force_update"] = True
        elif opt in ("-d", "--downDDC"):
            if arg in ["true", "True"]:
                param["is_down_ddc"] = True
        elif opt in ("-o", "--outputDir"):
            param["output_directory"] = str(arg)
        elif opt in ("-c", "--contentType"):
            param["content_type"] = str(arg)
        elif opt in ("-p", "--proxy"):
            PROXY_URL = str(arg)

    if version == "":
        print(
            "Error: set version correct. please select version from scene_manager/data/dataset-repo-update.log"
        )
        print_help()

    mode = get_version_process_mode(version)
    if mode == 'raw':
        if virtualworld_id == "":
            virtualworld_ids_file = os.path.join(
                os.path.dirname(__file__), "data/virtualworld-ids.json"
            )
            if os.path.exists(virtualworld_ids_file):
                with open(virtualworld_ids_file) as f:
                    for id in json.load(f):
                        download_single_virtualworld(
                            id, version, param["is_down_ddc"], param["is_force_update"]
                        )
        else:
            download_single_virtualworld(
                virtualworld_id, version, param["is_down_ddc"], param["is_force_update"]
            )
    elif mode == "pak":
        if virtualworld_id == "":
            virtualworld_ids_file = os.path.join(
                os.path.dirname(__file__), "data/virtualworld-ids.json"
            )
            if os.path.exists(virtualworld_ids_file):
                with open(virtualworld_ids_file) as f:
                    for id in json.load(f):
                        download_single_virtualworld_pak(
                            id, version, param
                        )
        else:
            download_single_virtualworld_pak(
                virtualworld_id, version, param
            )
    else:
        print(f"unknown process mode: {version}")
