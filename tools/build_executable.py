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
    parser.add_argument("--output_dir", default=os.path.dirname(os.path.realpath(__file__)))
    parser.add_argument("--temp_dir", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "tmp"))
    parser.add_argument("--conda_script")
    parser.add_argument("--commit_id")
    args = parser.parse_args()

    repo_dir           = os.path.join(os.path.realpath(args.temp_dir), "spear")
    unreal_project_dir = os.path.realpath(os.path.join(repo_dir, "cpp", "unreal_projects", "SpearSim"))
    unreal_plugins_dir = os.path.realpath(os.path.join(repo_dir, "cpp", "unreal_plugins"))

    # remove any contents already present for a clean build
    if os.path.exists(repo_dir):
        shutil.rmtree(repo_dir)
        os.makedirs(repo_dir)

    if sys.platform == "win32":
        run_uat_script = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.bat")
        target_platform = "Win64"
        archive_dir = os.path.join(args.output_dir, f"SpearSim-{target_platform}-{args.config_mode}")
        cmd_prefix = f"conda activate {args.conda_env}&"
    elif sys.platform == "darwin":
        run_uat_script = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh")
        target_platform = "Mac"
        archive_dir = os.path.join(args.output_dir, f"SpearSim-{target_platform}-{args.config_mode}-Unsigned")
        conda_script = args.conda_script
        if not conda_script:
            conda_script = "~/anaconda3/etc/profile.d/conda.sh"
        cmd_prefix = f". {conda_script}; conda activate {args.conda_env};"
    elif sys.platform == "linux":
        run_uat_script = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh")
        target_platform = "Linux"
        archive_dir = os.path.join(args.output_dir, f"SpearSim-{target_platform}-{args.config_mode}")
        conda_script = args.conda_script
        if not conda_script:
            conda_script = "~/anaconda3/etc/profile.d/conda.sh"
        cmd_prefix = f". {conda_script}; conda activate {args.conda_env};"
    else:
        assert False

    # clone repo along with submodules
    cmd = ["git", "clone", "--recurse-submodules", "https://github.com/isl-org/spear", repo_dir]
    print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, check=True)
    assert cmd_result.returncode == 0

    if args.commit_id:
        cmd = ["git", "reset", "--hard", args.commit_id]
        print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd, check=True)
        assert cmd_result.returncode == 0

    # build third-party libs
    cmd = ["python", "build_third_party_libs.py", "--num_parallel_jobs", f"{args.num_parallel_jobs}"]
    print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, check=True)
    assert cmd_result.returncode == 0

    # create symbolic links
    cmd = cmd_prefix + f"python create_symbolic_links.py --unreal_project_dir {unreal_project_dir} --unreal_plugins_dir {unreal_plugins_dir}"
    print(f"[SPEAR | build_executable.py] Executing: {cmd}")
    cmd_result = subprocess.run(cmd, shell=True, check=True)
    assert cmd_result.returncode == 0

    # generate config file
    cmd = cmd_prefix + f"python generate_config.py --unreal_project_dir {unreal_project_dir}"
    print(f"[SPEAR | build_executable.py] Executing: {cmd}")
    cmd_result = subprocess.run(cmd, shell=True, check=True)
    assert cmd_result.returncode == 0
    
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
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    print(f"[SPEAR | build_executable.py] Successfully built SpearSim at {archive_dir}.")
    print(f"[SPEAR | build_executable.py] Done.")
