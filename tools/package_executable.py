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
    parser.add_argument("--conda_env", required=True)
    parser.add_argument("--conda_path")
    parser.add_argument("--config_mode", default="Shipping")
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--num_parallel_jobs", type=int, default=1)
    parser.add_argument("--target_platform", required=True, help="Values accepted are Win64, Linux, Mac")
    parser.add_argument("--output_dir", default=os.path.dirname(os.path.realpath(__file__)))
    parser.add_argument("--output_tag", required=True)
    args = parser.parse_args()

    repo_dir = os.path.join(os.path.realpath(args.temp_dir), "spear")

    # need conda_path on unix systems
    if sys.platform in [ "darwin", "linux"]:
        assert args.conda_path

    # remove any contents already present for a clean build
    if os.path.exists(repo_dir):
        shutil.rmtree(repo_dir)
        os.makedirs(repo_dir)

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
    if sys.platform == "win32":
        cmd = ["python", "build_third_party_libs.py", "--num_parallel_jobs", f"{args.num_parallel_jobs}"]
    elif sys.platform in ["linux", "darwin"]:    
        cmd = [f"python build_third_party_libs.py --num_parallel_jobs {args.num_parallel_jobs}"]
    else:
        assert False
    print(f"[SPEAR | build.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

    # create symbolic links
    if sys.platform == "win32":
        cmd = ["conda", "activate", f"{args.conda_env}&", "python", "create_symbolic_links.py"]
    elif sys.platform in ["linux", "darwin"]:
        cmd = [f". {args.conda_path}; conda activate {args.conda_env}; python create_symbolic_links.py"]
    else:
        assert False
    print(f"[SPEAR | build.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

    # generate config file
    if sys.platform == "win32":
       cmd = ["conda", "activate", f"{args.conda_env}&", "python", "generate_config.py"]
    elif sys.platform in ["linux", "darwin"]: 
        cmd = [f". {args.conda_path}; conda activate {args.conda_env}; python generate_config.py"]
    else:
        assert False
    print(f"[SPEAR | build.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

    # build the executable
    if sys.platform == "win32":
        build_exe = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.bat")
    elif sys.platform in ["linux", "darwin"]:
        build_exe = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh")
    else:
        assert False
    
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
        "-archivedirectory=" + os.path.join(args.temp_dir, f"SpearSim-{args.config_mode}"),
        "-clientconfig=" + args.config_mode,
    ]
    print(f"[SPEAR | build.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    # change working directory
    print(f"[SPEAR | build.py] Changing working directory to {cwd}")
    os.chdir(cwd)

    print(f"[SPEAR | build.py] Successfully built SpearSim for {args.target_platform} platform with {args.config_mode} mode.")

    # zip the exectuable for distribution
    if sys.platform == "win32":
        platform_dir_name = "WindowsNoEditor"
    elif sys.platform == "darwin":
        platform_dir_name = "MacNoEditor"
    elif sys.platform == "linux":
        platform_dir_name = "LinuxNoEditor"
    else:
        assert False
    
    os.makedirs(args.output_dir, exist_ok=True)

    # TODO: need to codesign on macOS
    file = shutil.make_archive(base_name=os.path.join(args.output_dir, f"SpearSim-{args.output_tag}-{args.target_platform}-{args.config_mode}"), format='zip', root_dir=os.path.join(args.temp_dir, f"SpearSim-{args.config_mode}", platform_dir_name))
    shutil.rmtree(os.path.join(args.output_dir, platform_dir_name))

    print(f"[SPEAR | build.py] Successfully created a zip file at {file}, ready for distribution.")
