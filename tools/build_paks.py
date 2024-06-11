#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import fnmatch
import glob
import ntpath
import os
import posixpath
import spear
import subprocess
import sys


# globals needed in build_pak(...)
args = default_pak_asset_names = unreal_project_dir = unreal_project_content_dir = unreal_project_cooked_dir = None


def build_pak(pak_name, symlink_dirs=[], cooked_include_dirs=[], expected_unreal_project_content_scene_dirs=None):

    global args, default_pak_asset_names, unreal_project_dir, unreal_project_content_dir, unreal_project_cooked_dir

    spear.log(f"Building: {pak_name}")

    # input paths
    uproject = os.path.realpath(os.path.join(unreal_project_dir, "SpearSim.uproject"))
    unreal_project_content_scenes_dir = os.path.realpath(os.path.join(unreal_project_content_dir, "Scenes"))
    unreal_project_cooked_dir_posix = unreal_project_cooked_dir.replace(ntpath.sep, posixpath.sep)

    # output paths
    output_dir = os.path.realpath(args.output_dir)
    txt_file   = os.path.realpath(os.path.join(output_dir, pak_name + "-" + args.version_tag + "-" + platform + ".txt"))
    pak_file   = os.path.realpath(os.path.join(output_dir, pak_name + "-" + args.version_tag + "-" + platform + ".pak"))

    # create symlinks
    for symlink_dir_physical, symlink_dir_virtual in symlink_dirs.items():
        if spear.path_exists(symlink_dir_virtual):
            spear.log(f"    File or directory or symlink exists, removing: {symlink_dir_virtual}")
            spear.remove_path(symlink_dir_virtual)
        spear.log(f"    Creating symlink: {symlink_dir_virtual} -> {symlink_dir_physical}")
        os.symlink(symlink_dir_physical, symlink_dir_virtual)

    # now that we have created our symlinks, check that the Content/Scenes dir contains exactly expected_unreal_project_content_scene_dirs
    if expected_unreal_project_content_scene_dirs is not None:
        ignore_names = [".DS_Store"]
        unreal_project_content_scene_dirs = { os.path.basename(x) for x in os.listdir(unreal_project_content_scenes_dir) if x not in ignore_names }
        assert unreal_project_content_scene_dirs == set(expected_unreal_project_content_scene_dirs)

    # cook project, see https://docs.unrealengine.com/5.2/en-US/SharingAndReleasing/Deployment/Cooking for more details
    cmd = [
        unreal_editor_bin,
        uproject,
        "-run=Cook",
        "-targetplatform=" + platform,
        "-unattended",                           # don't require any user input
        "-iterate",                              # only cook content that needs to be updated
        "-fileopenlog",                          # generate a log of which files are opened in which order
        "-ddc=InstalledDerivedDataBackendGraph", # use the default cache location for installed (i.e., not source) builds of the engine
        "-unversioned",                          # save all of the cooked packages without versions
        "-stdout",                               # ensure log output is written to the terminal 
        "-fullstdoutlogoutput",                  # ensure log output is written to the terminal
        "-nologtimes"]                           # don't print timestamps next to log messages twice
    spear.log(f"    Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    # create the output_dir
    os.makedirs(output_dir, exist_ok=True)

    # create manifest file in output dir
    with open(txt_file, mode="w") as f:
        for cooked_include_dir in cooked_include_dirs:
            for cooked_include_file in glob.glob(os.path.realpath(os.path.join(cooked_include_dir, "**", "*.*")), recursive=True):

                cooked_include_file_posix = cooked_include_file.replace(ntpath.sep, posixpath.sep)
                assert cooked_include_file_posix.startswith(unreal_project_cooked_dir_posix)
                asset_name = cooked_include_file_posix.removeprefix(unreal_project_cooked_dir_posix + posixpath.sep)

                if asset_name not in default_pak_asset_names:
                    cooked_mount_file_posix = posixpath.join("..", "..", "..", cooked_include_file_posix.replace(unreal_project_cooked_dir_posix + posixpath.sep, ""))
                    f.write(f'"{cooked_include_file_posix}" "{cooked_mount_file_posix}" "" \n')

    # build pak file
    cmd = [unreal_pak_bin, pak_file, "-create=" + txt_file, "-platform=" + platform, "-multiprocess", "-compressed"]
    spear.log(f"    Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    assert os.path.exists(pak_file)
    spear.log(f"    Successfully built: {pak_file}")

    # remove symlinks
    for symlink_dir_physical, symlink_dir_virtual in symlink_dirs.items():
        spear.log(f"    Removing symlink: {symlink_dir_virtual}")
        spear.remove_path(symlink_dir_virtual)


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--perforce_content_dir", required=True)
    parser.add_argument("--unreal_project_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
    parser.add_argument("--build_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "BUILD")))
    parser.add_argument("--scene_ids")
    parser.add_argument("--skip_build_common_pak", action="store_true")
    args = parser.parse_args()

    assert os.path.exists(args.unreal_engine_dir)
    assert os.path.exists(args.perforce_content_dir)
    assert os.path.exists(args.unreal_project_dir)
    assert os.path.exists(args.build_dir)

    if sys.platform == "win32":
        platform          = "Windows"
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealEditor.exe"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealPak.exe"))
        default_pak       = os.path.realpath(os.path.join(args.build_dir, "SpearSim-Win64-Shipping", "Windows", "SpearSim", "Content", "Paks", "SpearSim-Windows.pak"))
    elif sys.platform == "darwin":
        platform          = "Mac"
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealEditor.app", "Contents", "MacOS", "UnrealEditor"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealPak"))
        default_pak       = os.path.realpath(os.path.join(args.build_dir, "SpearSim-Mac-Shipping-Unsigned", "Mac", "SpearSim-Mac-Shipping.app", "Contents", "UE", "SpearSim", "Content", "Paks", "SpearSim-Mac.pak"))
    elif sys.platform == "linux":
        platform          = "Linux"
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealEditor"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealPak"))
        default_pak       = os.path.realpath(os.path.join(args.build_dir, "SpearSim-Linux-Shipping", "Linux", "SpearSim", "Content", "Paks", "SpearSim-Linux.pak"))
    else:
        assert False

    unreal_project_dir                = os.path.realpath(args.unreal_project_dir)
    unreal_project_content_dir        = os.path.realpath(os.path.join(unreal_project_dir, "Content"))
    unreal_project_content_scenes_dir = os.path.realpath(os.path.join(unreal_project_content_dir, "Scenes"))
    unreal_project_cooked_dir         = os.path.realpath(os.path.join(unreal_project_dir, "Saved", "Cooked", platform))
    perforce_content_scenes_dir       = os.path.realpath(os.path.join(args.perforce_content_dir, "Scenes"))

    # extract asset names from the executable's pak file
    cmd = [unreal_pak_bin, "-List", default_pak]
    spear.log(f"    Executing: {' '.join(cmd)}")
    ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
    default_pak_asset_names = []
    for line in ps.stdout:
        default_pak_asset_names.append(line.removeprefix('LogPakFile: Display: "').split('" offset: ', 1)[0])
    ps.wait()
    ps.stdout.close()

    # We do not want to use os.path.realpath(...) for the values in symlink_dirs and cooked_include_dirs, because
    # we do not want these values to resolve to directories inside the user's Perforce workspace. Instead,
    # we want these values to refer to unresolved symlink directories in the user's Unreal project directory.
 
    # build common pak
    if not args.skip_build_common_pak:
        pak_name = "Common"
        symlink_dirs = {
            os.path.realpath(os.path.join(args.perforce_content_dir, "Megascans")) : os.path.join(unreal_project_content_dir, "Megascans"),
            os.path.realpath(os.path.join(args.perforce_content_dir, "MSPresets")) : os.path.join(unreal_project_content_dir, "MSPresets")}
        cooked_include_dirs = [
            os.path.join(unreal_project_cooked_dir, "Engine", "Content"),
            os.path.join(unreal_project_cooked_dir, "Engine", "Plugins"),
            os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "Megascans"),
            os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "MSPresets")]
        expected_unreal_project_content_scene_dirs = ["apartment_0000", "debug_0000", "debug_0001"]
        build_pak(pak_name, symlink_dirs, cooked_include_dirs, expected_unreal_project_content_scene_dirs)

    # get scene ids for building per-scene paks
    ignore_names = [".DS_Store"]
    candidate_scene_ids = [ os.path.basename(x) for x in os.listdir(perforce_content_scenes_dir) if x not in ignore_names ]

    if args.scene_ids is None:
        scene_ids = candidate_scene_ids
    else:
        arg_scene_id_strings = args.scene_ids.split(",")
        scene_ids = []
        for arg_scene_id_string in arg_scene_id_strings:
            scene_ids.extend([ candidate_scene_id for candidate_scene_id in candidate_scene_ids if fnmatch.fnmatch(candidate_scene_id, arg_scene_id_string) ])

    # build per-scene paks
    for scene_id in scene_ids:
        pak_name = scene_id
        symlink_dirs = {
            os.path.realpath(os.path.join(args.perforce_content_dir, "Megascans")) : os.path.join(unreal_project_content_dir, "Megascans"),
            os.path.realpath(os.path.join(args.perforce_content_dir, "MSPresets")) : os.path.join(unreal_project_content_dir, "MSPresets"),
            os.path.realpath(os.path.join(perforce_content_scenes_dir, scene_id))  : os.path.join(unreal_project_content_scenes_dir, scene_id)}
        cooked_include_dirs = [
            os.path.realpath(os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "Scenes", scene_id))]
        expected_unreal_project_content_scene_dirs = ["apartment_0000", "debug_0000", "debug_0001", scene_id]
        build_pak(pak_name, symlink_dirs, cooked_include_dirs, expected_unreal_project_content_scene_dirs)

    spear.log("Done.")
