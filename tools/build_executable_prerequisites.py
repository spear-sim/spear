#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import spear
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--unreal-engine-dir", required=True)
parser.add_argument("--commit-id")
parser.add_argument("--skip-clone-github-repo", action="store_true")
parser.add_argument("--skip-build-third-party-libs", action="store_true")
parser.add_argument("--skip-copy-engine-content", action="store_true")
parser.add_argument("--build-dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "BUILD")))
parser.add_argument("--conda-env", default="spear-env")
args = parser.parse_args()


if __name__ == "__main__":

    repo_dir           = os.path.realpath(os.path.join(args.build_dir, "spear"))
    unreal_project_dir = os.path.realpath(os.path.join(repo_dir, "cpp", "unreal_projects", "SpearSim"))
    third_party_dir    = os.path.realpath(os.path.join(repo_dir, "third_party"))

    spear.tools.validate_build_environment()

    if not args.skip_clone_github_repo:

        # remove our temporary repository directory to ensure a clean build
        if os.path.exists(repo_dir):
            spear.log("Repository exists, removing: ", repo_dir)
            shutil.rmtree(repo_dir, ignore_errors=True)

        # clone repo with submodules
        cmd = ["git", "clone", "--recurse-submodules", "https://github.com/spear-sim/spear", repo_dir]
        spear.tools.run_cmd(cmd=cmd)

        # reset to a specific commit ID
        if args.commit_id:
            cwd = os.getcwd()
            os.chdir(repo_dir)
            cmd = ["git", "reset", "--hard", args.commit_id]
            spear.tools.run_cmd(cmd=cmd)
            os.chdir(cwd)

    if not args.skip_build_third_party_libs:

        if sys.platform == "win32":
            build_third_party_libs_platform_args = []
        elif sys.platform == "darwin":
            build_third_party_libs_platform_args = []
        elif sys.platform == "linux":
            build_third_party_libs_platform_args = ["--unreal-engine-dir", args.unreal_engine_dir]
        else:
            assert False

        # build third-party libs
        cmd = [
            "python",
            os.path.realpath(os.path.join(os.path.dirname(__file__), "build_third_party_libs.py")),
            "--third-party-dir", third_party_dir] + \
            build_third_party_libs_platform_args
        spear.tools.run_conda_cmd(conda_env=args.conda_env, cmd=cmd)

    if not args.skip_copy_engine_content:

        # copy starter content
        cmd = [
            "python",
            os.path.realpath(os.path.join(os.path.dirname(__file__), "copy_engine_content.py")),
            "--unreal-engine-dir", args.unreal_engine_dir,
            "--unreal-project-dir", unreal_project_dir]
        spear.tools.run_conda_cmd(conda_env=args.conda_env, cmd=cmd)

    spear.log("Done.")
