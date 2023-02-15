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
    parser.add_argument("--temp_dir", required=True)
    parser.add_argument("--output_tag", required=True)
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--target_platform", required=True)
    parser.add_argument("--shell_bin")
    parser.add_argument("--conda_env", default="spear-env")
    parser.add_argument("--config_mode", default="Shipping")
    parser.add_argument("--num_parallel_jobs", type=int, default=1)
    parser.add_argument("--output_dir", default=os.path.dirname(os.path.realpath(__file__)))
    args = parser.parse_args()

    repo_dir = os.path.join(os.path.realpath(args.temp_dir), "spear")

    # remove any contents already present for a clean build
    if os.path.exists(repo_dir):
        shutil.rmtree(repo_dir)
        os.makedirs(repo_dir)

    if sys.platform == "win32":
        cmd_separator = "&"
        build_exe = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.bat")
    elif sys.platform == "darwin":
        cmd_separator = ";"
        build_exe = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh")
    elif sys.platform == "linux":
        cmd_separator = ";"
        build_exe = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh")
    else:
        assert False

    # clone repo along with submodules  
    cmd = ["git", "clone", "--recurse-submodules", "https://github.com/isl-org/spear", repo_dir]
    print(f"[SPEAR | build.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    # change working directory
    cwd = os.getcwd()
    print(f"[SPEAR | build.py] Changing working directory to {os.path.join(repo_dir, 'tools')}")
    os.chdir(os.path.join(repo_dir, "tools"))

    # build thirdparty libs
    cmd = ["python", "build_third_party_libs.py", "--num_parallel_jobs", f"{args.num_parallel_jobs}"]
    print(f"[SPEAR | build.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

    # create symbolic links
    cmd = ["conda", "activate", args.conda_env + cmd_separator, "python", "create_symbolic_links.py"]
    print(f"[SPEAR | build.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True, executable=args.shell_bin)
    assert cmd_result.returncode == 0

    # generate config file
    cmd = ["conda", "activate", args.conda_env + cmd_separator, "python", "generate_config.py"]
    print(f"[SPEAR | build.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True, executable=args.shell_bin)
    assert cmd_result.returncode == 0
    
    cmd = [
        build_exe,
        "BuildCookRun",
        "-project=" + os.path.join(repo_dir, "cpp", "unreal_projects", "SpearSim", "SpearSim.uproject"), 
        "-build",
        "-cook",
        "-stage",
        "-package",
        "-archive",
        "-pak",
        "-targetplatform=" + args.target_platform,
        "-target=SpearSim",
        "-archivedirectory=" + os.path.join(args.output_dir, f"SpearSim-{args.output_tag}-{args.config_mode}"),
        "-clientconfig=" + args.config_mode
    ]
    print(f"[SPEAR | build.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    # change working directory
    print(f"[SPEAR | build.py] Changing working directory to {cwd}")
    os.chdir(cwd)

    print(f"[SPEAR | build.py] Successfully built SpearSim for {args.target_platform} platform with {args.config_mode} mode.")
