#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import spear
import subprocess
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--unreal_engine_dir", required=True)
parser.add_argument("--commit_id")
parser.add_argument("--conda_script")
parser.add_argument("--skip_clone_github_repo", action="store_true")
parser.add_argument("--skip_build_third_party_libs", action="store_true")
parser.add_argument("--skip_copy_engine_content", action="store_true")
parser.add_argument("--build_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "BUILD")))
parser.add_argument("--conda_env", default="spear-env")
args = parser.parse_args()


if __name__ == "__main__":

    repo_dir           = os.path.realpath(os.path.join(args.build_dir, "spear"))
    unreal_project_dir = os.path.realpath(os.path.join(repo_dir, "cpp", "unreal_projects", "SpearSim"))
    third_party_dir    = os.path.realpath(os.path.join(repo_dir, "third_party"))

    # set various platform-specific variables that we use throughout our build procedure
    if sys.platform == "win32":
        target_platform = "Win64"
        cmd_prefix      = f"conda activate {args.conda_env} & "

    elif sys.platform == "darwin":
        target_platform = "Mac"

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

    if not args.skip_copy_engine_content:

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

    spear.log("Done.")
