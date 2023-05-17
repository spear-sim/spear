#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import spear
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--paks_dir", required=True)
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--conda_env", default="spear-env")
    parser.add_argument("--num_parallel_jobs", type=int, default=1)
    parser.add_argument("--output_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "tmp")))
    parser.add_argument("--temp_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "tmp")))
    parser.add_argument("--conda_script")
    parser.add_argument("--commit_id")
    parser.add_argument("--skip_clone_github_repo", action="store_true")
    parser.add_argument("--skip_build_third_party_libs", action="store_true")
    args = parser.parse_args()

    assert os.path.exists(args.unreal_engine_dir)
    assert os.path.exists(args.paks_dir)

    repo_dir           = os.path.realpath(os.path.join(args.temp_dir, "spear"))
    unreal_project_dir = os.path.realpath(os.path.join(repo_dir, "cpp", "unreal_projects", "SpearSim"))
    unreal_plugins_dir = os.path.realpath(os.path.join(repo_dir, "cpp", "unreal_plugins"))
    third_party_dir    = os.path.realpath(os.path.join(repo_dir, "third_party"))
    uproject           = os.path.realpath(os.path.join(unreal_project_dir, "SpearSim.uproject"))
    build_config       = "Shipping"

    # set various platform-specific variables that we use throughout our build procedure
    if sys.platform == "win32":
        target_platform  = "Win64"
        pak_platform     = "Windows"
        run_uat_script   = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.bat"))
        archive_dir      = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{target_platform}-{build_config}"))
        config_file_src  = os.path.realpath(os.path.join(os.path.dirname(__file__), "WindowsEngine.ini"))
        config_file_dest = os.path.realpath(os.path.join(unreal_project_dir, "Config", "Windows", "WindowsEngine.ini"))
        paks_dir_dest    = os.path.realpath(os.path.join(archive_dir, "WindowsNoEditor", "SpearSim", "Content", "Paks"))
        unreal_tmp_dir   = ""
        cmd_prefix       = f"conda activate {args.conda_env}& "

    elif sys.platform == "darwin":
        target_platform  = "Mac"
        pak_platform     = "Mac"
        run_uat_script   = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh"))
        archive_dir      = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{target_platform}-{build_config}-Unsigned"))
        config_file_src  = os.path.realpath(os.path.join(os.path.dirname(__file__), "MacEngine.ini"))
        config_file_dest = os.path.realpath(os.path.join(unreal_project_dir, "Config", "Mac", "MacEngine.ini"))
        paks_dir_dest    = os.path.realpath(os.path.join(archive_dir, "MacNoEditor", "SpearSim-Mac-Shipping.app", "Contents", "UE4", "SpearSim", "Content", "Paks"))
        unreal_tmp_dir   = os.path.expanduser(os.path.join("~", "Library", "Preferences", "Unreal Engine", "SpearSimEditor"))

        if args.conda_script:
            conda_script = args.conda_script
        else:
            # default for graphical install, see https://docs.anaconda.com/anaconda/user-guide/faq
            conda_script = os.path.expanduser(os.path.join("~", "opt", "anaconda3", "etc", "profile.d", "conda.sh"))

        cmd_prefix = f". {conda_script}; conda activate {args.conda_env}; "

    elif sys.platform == "linux":
        target_platform  = "Linux"
        pak_platform     = "Linux"
        run_uat_script   = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh"))        
        archive_dir      = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{target_platform}-{build_config}"))
        config_file_src  = os.path.realpath(os.path.join(os.path.dirname(__file__), "LinuxEngine.ini"))
        config_file_dest = os.path.realpath(os.path.join(unreal_project_dir, "Config", "Linux", "LinuxEngine.ini"))
        paks_dir_dest    = os.path.realpath(os.path.join(archive_dir, "LinuxNoEditor", "SpearSim", "Content", "Paks"))
        unreal_tmp_dir   = ""

        if args.conda_script:
            conda_script = args.conda_script
        else:
            # see https://docs.anaconda.com/anaconda/user-guide/faq
            conda_script = os.path.expanduser(os.path.join("~", "anaconda3", "etc", "profile.d", "conda.sh"))

        cmd_prefix = f". {conda_script}; conda activate {args.conda_env}; "

    else:
        assert False

    assert os.path.exists(run_uat_script)

    # once we know pak_platform, set our default pak file src and dest
    pak_file_src  = os.path.realpath(os.path.join(args.paks_dir, "kujiale_0000-" + args.version_tag + "-" + pak_platform + ".pak"))
    pak_file_dest = os.path.realpath(os.path.join(paks_dir_dest, "kujiale_0000-" + args.version_tag + "-" + pak_platform + ".pak"))

    # We need to remove this temp dir (created by the Unreal build process) because it contains paths from previous builds.
    # If we don't do this step, we will get many warnings during this build:
    #     Warning: Unable to generate long package name for path/to/previous/build/Some.uasset because FilenameToLongPackageName failed to convert
    #     'path/to/previous/build/Some.uasset'. Attempt result was '../../../../../../path/to/previous/build/path/to/previous/build/Some', but the
    #     path contains illegal characters '.'
    if os.path.exists(unreal_tmp_dir):
        spear.log(f"Unreal Engine cache directory exists, removing: {unreal_tmp_dir}")
        shutil.rmtree(unreal_tmp_dir, ignore_errors=True)

    if not args.skip_clone_github_repo:
        
        # remove our temporary repository directory to ensure a clean build
        if os.path.exists(repo_dir):
            spear.log(f"Repository exists, removing: {repo_dir}")
            shutil.rmtree(repo_dir, ignore_errors=True)

        # clone repo with submodules
        cmd = ["git", "clone", "--recurse-submodules", "https://github.com/isl-org/spear", repo_dir]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        # reset to a specific commit ID
        if args.commit_id:
            cwd = os.getcwd()
            os.chdir(repo_dir)
            cmd = ["git", "reset", "--hard", args.commit_id]
            spear.log(f"Executing: {' '.join(cmd)}")
            subprocess.run(cmd, check=True)
            os.chdir(cwd)

    if not args.skip_build_third_party_libs:

        # build third-party libs
        cmd = [
            "python",
            os.path.join("..", "build_third_party_libs.py"),
            "--third_party_dir", third_party_dir,
            "--num_parallel_jobs", f"{args.num_parallel_jobs}"
        ]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    # create symbolic links (we need shell=True because we want to run in a specific anaconda env,
    # and we need to break up this string extra carefully so we can enclose unreal_engine_dir in
    # quotes, since it will often have spaces in its path on Windows)
    cmd = \
        cmd_prefix + \
        "python " + \
        f"{os.path.join('..', 'create_symbolic_links.py')} " + \
        f'--unreal_engine_dir "{args.unreal_engine_dir}" ' + \
        f"--unreal_project_dir {unreal_project_dir} " + \
        f"--unreal_plugins_dir {unreal_plugins_dir} " \
        f"--third_party_dir {third_party_dir}"
    spear.log(f"Executing: {cmd}")
    subprocess.run(cmd, shell=True, check=True)

    # copy our custom ini file, used to update the default map for public release executables
    shutil.copyfile(config_file_src, config_file_dest)
    spear.log(f"Copied {config_file_src} to {config_file_dest}")

    # build SpearSim project
    cmd = [
        run_uat_script,
        "BuildCookRun",
        "-project=" + os.path.realpath(os.path.join(unreal_project_dir, "SpearSim.uproject")),
        "-build",            # build C++ executable
        "-cook",             # prepare cross-platform content for a specific platform (e.g., compile shaders)
        "-stage",            # copy C++ executable to a staging directory (required for -pak)
        "-package",          # prepare staged executable for distribution on a specific platform (e.g., runs otool and xcrun on macOS)
        "-archive",          # copy staged executable to -archivedirectory (on some platforms this will also move directories around relative to the executable)
        "-pak",              # generate a pak file for cooked content and configure executable so it can load pak files
        "-iterativecooking", # only cook content that needs to be updated
        "-targetplatform=" + target_platform,
        "-target=SpearSim",
        "-archivedirectory=" + archive_dir,
        "-clientconfig=" + build_config,
    ]
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    # copy our custom pak file
    shutil.copyfile(pak_file_src, pak_file_dest)
    spear.log(f"Copied {pak_file_src} to {pak_file_dest}")

    # We need to remove this temp dir (created by the Unreal build process) because it contains paths from the above build.
    # If we don't do this step, we will get many warnings during subsequent builds:
    #     Warning: Unable to generate long package name for path/to/previous/build/Some.uasset because FilenameToLongPackageName failed to convert
    #     'path/to/previous/build/Some.uasset'. Attempt result was '../../../../../../path/to/previous/build/path/to/previous/build/Some', but the
    #     path contains illegal characters '.'
    if os.path.exists(unreal_tmp_dir):
        spear.log(f"Unreal Engine cache directory exists, removing: {unreal_tmp_dir}")
        shutil.rmtree(unreal_tmp_dir, ignore_errors=True)

    spear.log(f"Successfully built SpearSim at {archive_dir}")
    spear.log("Done.")
