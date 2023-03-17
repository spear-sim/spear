#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--conda_env", default="spear-env")
    parser.add_argument("--config_mode", default="Shipping")
    parser.add_argument("--num_parallel_jobs", type=int, default=1)
    parser.add_argument("--output_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "tmp")))
    parser.add_argument("--temp_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "tmp")))
    parser.add_argument("--conda_script")
    parser.add_argument("--commit_id")
    args = parser.parse_args()

    repo_dir           = os.path.realpath(os.path.join(args.temp_dir, "spear"))
    unreal_project_dir = os.path.realpath(os.path.join(repo_dir, "cpp", "unreal_projects", "SpearSim"))
    unreal_plugins_dir = os.path.realpath(os.path.join(repo_dir, "cpp", "unreal_plugins"))
    third_party_dir    = os.path.realpath(os.path.join(repo_dir, "third_party"))

    # set various platform-specific variables that we use throughout our build procedure
    if sys.platform == "win32":
        run_uat_script  = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.bat"))
        target_platform = "Win64"
        archive_dir     = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{target_platform}-{args.config_mode}"))
        cmd_prefix      = f"conda activate {args.conda_env}& "
        unreal_tmp_dir  = ""

    elif sys.platform == "darwin":
        run_uat_script  = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh"))
        target_platform = "Mac"
        archive_dir     = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{target_platform}-{args.config_mode}-Unsigned"))
        unreal_tmp_dir  = os.path.expanduser(os.path.join("~", "Library", "Preferences", "Unreal Engine", "SpearSimEditor"))

        if args.conda_script:
            conda_script = args.conda_script
        else:
            # default for graphical install, see https://docs.anaconda.com/anaconda/user-guide/faq/
            conda_script = os.path.expanduser(os.path.join("~", "opt", "anaconda3", "etc", "profile.d", "conda.sh"))

        cmd_prefix = f". {conda_script}; conda activate {args.conda_env}; "

    elif sys.platform == "linux":
        run_uat_script  = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh"))
        target_platform = "Linux"
        archive_dir     = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{target_platform}-{args.config_mode}"))
        unreal_tmp_dir  = ""

        if args.conda_script:
            conda_script = args.conda_script
        else:
            # see https://docs.anaconda.com/anaconda/user-guide/faq/
            conda_script = os.path.expanduser(os.path.join("~", "anaconda3", "etc", "profile.d", "conda.sh"))

        cmd_prefix = f". {conda_script}; conda activate {args.conda_env}; "

    else:
        assert False

    # remove our temporary repository directory to ensure a clean build
    if os.path.exists(repo_dir):
        print(f"[SPEAR | build_executable.py] Repository exists, removing: {repo_dir}")
        shutil.rmtree(repo_dir)
        os.makedirs(repo_dir)

    # We need to remove this temp dir (created by the Unreal build process) because it contains paths from previous builds.
    # If we don't do this step, we will get many warnings during this build:
    #     Warning: Unable to generate long package name for path/to/previous/build/Some.uasset because FilenameToLongPackageName failed to convert
    #     'path/to/previous/build/Some.uasset'. Attempt result was '../../../../../../path/to/previous/build/path/to/previous/build/Some', but the
    #     path contains illegal characters '.'
    if os.path.exists(unreal_tmp_dir):
        print(f"[SPEAR | build_executable.py] Unreal Engine cache directory exists, removing: {unreal_tmp_dir}")
        shutil.rmtree(unreal_tmp_dir)

    # clone repo with submodules
    cmd = ["git", "clone", "--recurse-submodules", "https://github.com/isl-org/spear", repo_dir]
    print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    # reset to a specific commit ID
    if args.commit_id:
        cmd = ["git", "reset", "--hard", args.commit_id]
        print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    # build third-party libs
    cmd = ["python", "build_third_party_libs.py", "--third_party_dir", third_party_dir, "--num_parallel_jobs", f"{args.num_parallel_jobs}"]
    print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    # create symbolic links (need shell=True because we want to run in a specific anaconda env)
    cmd = cmd_prefix + f"python create_symbolic_links.py --unreal_project_dir {unreal_project_dir} --unreal_plugins_dir {unreal_plugins_dir} --third_party_dir {third_party_dir}"
    print(f"[SPEAR | build_executable.py] Executing: {cmd}")
    subprocess.run(cmd, shell=True, check=True)

    # generate config file (need shell=True because we want to run in a specific anaconda env)
    cmd = cmd_prefix + f"python generate_config.py --unreal_project_dir {unreal_project_dir}"
    print(f"[SPEAR | build_executable.py] Executing: {cmd}")
    subprocess.run(cmd, shell=True, check=True)

    # build SpearSim project
    cmd = [
        run_uat_script,
        "BuildCookRun",
        "-project=" + os.path.join(unreal_project_dir, "SpearSim.uproject"),
        "-build",
        "-cook",
        "-stage",
        "-package",
        "-archive",
        "-pak",
        "-targetplatform=" + target_platform,
        "-target=SpearSim",
        "-archivedirectory=" + archive_dir,
        "-clientconfig=" + args.config_mode
    ]
    print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    # We need to remove this temp dir (created by the Unreal build process) because it contains paths from the above build.
    # If we don't do this step, we will get many warnings during subsequent builds:
    #     Warning: Unable to generate long package name for path/to/previous/build/Some.uasset because FilenameToLongPackageName failed to convert
    #     'path/to/previous/build/Some.uasset'. Attempt result was '../../../../../../path/to/previous/build/path/to/previous/build/Some', but the
    #     path contains illegal characters '.'
    if os.path.exists(unreal_tmp_dir):
        print(f"[SPEAR | build_executable.py] Unreal Engine cache directory exists, removing: {unreal_tmp_dir}")
        shutil.rmtree(unreal_tmp_dir)

    print(f"[SPEAR | build_executable.py] Successfully built SpearSim at {archive_dir}")
    print("[SPEAR | build_executable.py] Done.")
