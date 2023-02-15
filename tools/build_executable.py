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
    parser.add_argument("--temp_dir", default=os.path.join(os.path.dirname(os.path.realpath(__file__), "tmp")))
    parser.add_argument("--commit_id")
    parser.add_argument("--shell_bin")
    args = parser.parse_args()

    repo_dir = os.path.join(os.path.realpath(args.temp_dir), "spear")

    # remove any contents already present for a clean build
    if os.path.exists(repo_dir):
        shutil.rmtree(repo_dir)
        os.makedirs(repo_dir)

    if sys.platform == "win32":
        cmd_separator = "&"
        run_uat_script = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.bat")
        target_platform = "Win64"
        archive_dir = os.path.join(args.output_dir, f"SpearSim-{target_platform}-{args.config_mode}")
    elif sys.platform == "darwin":
        cmd_separator = ";"
        run_uat_script = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh")
        target_platform = "Mac"
        archive_dir = os.path.join(args.output_dir, f"SpearSim-{target_platform}-{args.config_mode}-Unsigned")
    elif sys.plaform == "linux":
        cmd_separator = ";"
        run_uat_script = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh")
        target_platform = "Linux"
        archive_dir = os.path.join(args.output_dir, f"SpearSim-{target_platform}-{args.config_mode}")
    else:
        assert False

    # clone repo along with submodules  
    cmd = ["git", "clone", "--recurse-submodules", "https://github.com/isl-org/spear", repo_dir]
    print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    # change working directory
    cwd = os.getcwd()
    print(f"[SPEAR | build_executable.py] Changing working directory to {os.path.join(repo_dir, 'tools')}")
    os.chdir(os.path.join(repo_dir, "tools"))

    if args.commit_id:
        cmd = ["git", "reset", "--hard", args.commit_id]
        print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

    # build thirdparty libs
    cmd = ["python", "build_third_party_libs.py", "--num_parallel_jobs", f"{args.num_parallel_jobs}"]
    print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True, executable=args.shell_bin)
    assert cmd_result.returncode == 0

    # create symbolic links
    cmd = ["conda", "activate", args.conda_env + cmd_separator, "python", "create_symbolic_links.py"]
    print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True, executable=args.shell_bin)
    assert cmd_result.returncode == 0

    # generate config file
    cmd = ["conda", "activate", args.conda_env + cmd_separator, "python", "generate_config.py"]
    print(f"[SPEAR | build_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True, executable=args.shell_bin)
    assert cmd_result.returncode == 0
    
    # build SpearSim project
    cmd = [
        run_uat_script,
        "BuildCookRun",
        "-project=" + os.path.join(repo_dir, "cpp", "unreal_projects", "SpearSim", "SpearSim.uproject"), 
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

    # change working directory
    print(f"[SPEAR | build_executable.py] Changing working directory to {cwd}")
    os.chdir(cwd)

    print(f"[SPEAR | build_executable.py] Successfully built SpearSim at {archive_dir}.")
    print(f"[SPEAR | build_executable.py] Done.")
