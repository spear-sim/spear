#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import spear
import spear.tool_utils
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--commit_id")
    parser.add_argument("--conda_script")
    parser.add_argument("--skip_build_third_party_libs", action="store_true")
    parser.add_argument("--skip_clone_github_repo", action="store_true")
    parser.add_argument("--build_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "BUILD")))
    parser.add_argument("--conda_env", default="spear-env")
    args = parser.parse_args()

    assert os.path.exists(args.unreal_engine_dir)

    repo_dir           = os.path.realpath(os.path.join(args.build_dir, "spear"))
    unreal_project_dir = os.path.realpath(os.path.join(repo_dir, "cpp", "unreal_projects", "SpearSim"))
    unreal_plugins_dir = os.path.realpath(os.path.join(repo_dir, "cpp", "unreal_plugins"))
    third_party_dir    = os.path.realpath(os.path.join(repo_dir, "third_party"))
    uproject           = os.path.realpath(os.path.join(unreal_project_dir, "SpearSim.uproject"))
    build_config       = "Shipping"

    # set various platform-specific variables that we use throughout our build procedure
    if sys.platform == "win32":
        target_platform       = "Win64"
        run_uat_script        = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.bat"))
        archive_dir           = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-{target_platform}-{build_config}"))
        run_uat_platform_args = ""
        unreal_tmp_dir        = ""
        cmd_prefix            = f"conda activate {args.conda_env} & "

    elif sys.platform == "darwin":
        target_platform       = "Mac"
        run_uat_script        = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh"))
        archive_dir           = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-{target_platform}-{build_config}-Unsigned"))
        run_uat_platform_args = "-specifiedarchitecture=arm64+x86_64"
        unreal_tmp_dir        = os.path.expanduser(os.path.join("~", "Library", "Preferences", "Unreal Engine", "SpearSimEditor"))

        if args.conda_script:
            if os.path.exists(args.conda_script):
                spear.log("Found conda script at: ", args.conda_script)
                conda_script = args.conda_script
            assert conda_script is not None

        else:
            # see https://docs.anaconda.com/anaconda/user-guide/faq
            conda_script_candidates = [
                os.path.expanduser(os.path.join("~", "anaconda3", "etc", "profile.d", "conda.sh")),  # anaconda shell install
                os.path.join(os.sep, "opt", "anaconda3", "etc", "profile.d", "conda.sh"),            # anaconda graphical install
                os.path.expanduser(os.path.join("~", "miniconda3", "etc", "profile.d", "conda.sh")), # miniconda shell install
                os.path.join(os.sep, "opt", "miniconda3", "etc", "profile.d", "conda.sh")]           # miniconda graphical install

            conda_script = None
            for conda_script_candidate in conda_script_candidates:
                if os.path.exists(conda_script_candidate):
                    spear.log("Found conda script at: ", conda_script_candidate)
                    conda_script = conda_script_candidate
                    break
            assert conda_script is not None

        cmd_prefix = f". {conda_script}; conda activate {args.conda_env}; "

    elif sys.platform == "linux":
        target_platform = "Linux"
        run_uat_script        = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh"))        
        archive_dir           = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-{target_platform}-{build_config}"))
        run_uat_platform_args = ""
        unreal_tmp_dir        = ""

        if args.conda_script:
            if os.path.exists(args.conda_script):
                spear.log("Found conda script at: ", args.conda_script)
                conda_script = args.conda_script
            assert conda_script is not None

            # see https://docs.anaconda.com/anaconda/user-guide/faq
            conda_script_candidates = [
                os.path.expanduser(os.path.join("~", "anaconda3", "etc", "profile.d", "conda.sh")),
                os.path.expanduser(os.path.join("~", "miniconda3", "etc", "profile.d", "conda.sh"))]

            conda_script = None
            for conda_script_candidate in conda_script_candidates:
                if os.path.exists(conda_script_candidate):
                    spear.log("Found conda script at: ", conda_script_candidate)
                    conda_script = conda_script_candidate
                    break
            assert conda_script is not None

        cmd_prefix = f". {conda_script}; conda activate {args.conda_env}; "

    else:
        assert False

    assert os.path.exists(run_uat_script)

    # We need to remove this temp dir (created by the Unreal build process) because it contains paths from previous builds.
    # If we don't do this step, we will get many warnings during this build:
    #     Warning: Unable to generate long package name for path/to/previous/build/Some.uasset because FilenameToLongPackageName failed to convert
    #     'path/to/previous/build/Some.uasset'. Attempt result was '../../../../../../path/to/previous/build/path/to/previous/build/Some', but the
    #     path contains illegal characters '.'
    if os.path.exists(unreal_tmp_dir):
        spear.log("Unreal Engine cache directory exists, removing: ", unreal_tmp_dir)
        shutil.rmtree(unreal_tmp_dir, ignore_errors=True)

    if not args.skip_clone_github_repo:

        # remove our temporary repository directory to ensure a clean build
        if os.path.exists(repo_dir):
            spear.log("Repository exists, removing: ", repo_dir)
            shutil.rmtree(repo_dir, ignore_errors=True)

        # clone repo with submodules
        cmd = ["git", "clone", "--recurse-submodules", "https://github.com/spear-sim/spear", repo_dir]
        spear.log("Executing: ", " ".join(cmd))
        subprocess.run(cmd, check=True)

        # reset to a specific commit ID
        if args.commit_id:
            cwd = os.getcwd()
            os.chdir(repo_dir)
            cmd = ["git", "reset", "--hard", args.commit_id]
            spear.log("Executing: ", " ".join(cmd))
            subprocess.run(cmd, check=True)
            os.chdir(cwd)

    if not args.skip_build_third_party_libs:

        # build third-party libs (we need shell=True because we want to run in a specific anaconda env)
        cmd = \
            cmd_prefix + \
            "python " + \
            f'"{os.path.realpath(os.path.join(os.path.dirname(__file__), "build_third_party_libs.py"))}" ' + \
            f"--third_party_dir  {third_party_dir} "
        spear.log("Executing: ", cmd)
        subprocess.run(cmd, shell=True, check=True)

    # copy starter content (we need shell=True because we want to run in a specific anaconda env,
    # and we need to break up this string extra carefully so we can enclose unreal_engine_dir in
    # quotes, since it will often have spaces in its path on Windows)
    cmd = \
        cmd_prefix + \
        "python " + \
        f'"{os.path.realpath(os.path.join(os.path.dirname(__file__), "copy_engine_content.py"))}" ' + \
        f'--unreal_engine_dir "{args.unreal_engine_dir}" ' + \
        f'--unreal_project_dir "{unreal_project_dir}"'
    spear.log("Executing: ", cmd)
    subprocess.run(cmd, shell=True, check=True)

    # build SpearSim project

    cook_maps = spear.tool_utils.get_default_maps_to_cook()
    cook_maps_arg = [f"-map={"+".join(cook_maps)}"]

    cmd = [
        run_uat_script,
        "BuildCookRun",
        f'-project="{os.path.realpath(os.path.join(unreal_project_dir, "SpearSim.uproject"))}"',
        "-target=SpearSim",
        "-build",   # build C++ executable
        "-cook",    # prepare cross-platform content for a specific platform (e.g., compile shaders)
        "-stage",   # copy C++ executable to a staging directory (required for -pak)
        "-package", # prepare staged executable for distribution on a specific platform (e.g., runs otool and xcrun on macOS)
        "-archive", # copy staged executable to -archivedirectory (on some platforms this will also move directories around relative to the executable)
        "-pak",     # generate a pak file for cooked content and configure executable so it can load pak files
        f"-targetplatform={target_platform}",
        f"-clientconfig={build_config}",
        f'-archivedirectory="{archive_dir}"',
        run_uat_platform_args] + \
        cook_maps_arg
    spear.log("Executing: ", " ".join(cmd))
    subprocess.run(cmd, check=True)

    # We need to remove this temp dir (created by the Unreal build process) because it contains paths from the above build.
    # If we don't do this step, we will get many warnings during subsequent builds:
    #     Warning: Unable to generate long package name for path/to/previous/build/Some.uasset because FilenameToLongPackageName failed to convert
    #     'path/to/previous/build/Some.uasset'. Attempt result was '../../../../../../path/to/previous/build/path/to/previous/build/Some', but the
    #     path contains illegal characters '.'
    if os.path.exists(unreal_tmp_dir):
        spear.log("Unreal Engine cache directory exists, removing: ", unreal_tmp_dir)
        shutil.rmtree(unreal_tmp_dir, ignore_errors=True)

    spear.log(f"Successfully built SpearSim at {archive_dir}")
    spear.log("Done.")
