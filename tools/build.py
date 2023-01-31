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
    parser.add_argument("--repo_dir", required=True)
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--num_parallel_jobs", type=int, default=1)
    parser.add_argument("--target_platform", required=True, help="Values accepted are Win64, Linux, Mac")
    parser.add_argument("--config_mode", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--tag", required=True)
    args = parser.parse_args()

    # clone repo along with submodules
    cmd = ["git", "clone", "--recurse-submodules", "https://github.com/isl-org/spear", args.repo_dir]
    print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    os.makedirs(args.output_dir, exist_ok=True)

    # change working directory to the cloned directory above
    cwd = os.getcwd()
    print(f"[SPEAR | build_spear.py] Changing current working dir from {cwd} to {args.repo_dir}")
    os.chdir(args.repo_dir)

    # create a new virtual python env called spear-env
    if sys.platform == "win32":
        cmd = ["conda", "create", "-n", "spear-env", "python=3.8", "-y"]
    else:
        cmd = ["conda create -n spear-env python=3.8 -y"]
    print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

    # install msgpack-rpc-python package in the newly created python env
    if sys.platform == "win32":
        cmd = ["conda", "activate", "spear-env&", "pip", "install", "-e", "third_party/msgpack-rpc-python"]
    else:    
        cmd = ["conda activate spear-env; pip install -e third_party/msgpack-rpc-python"]
    print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

    # install spear-sim and dependent packages in the newly created python env
    if sys.platform == "win32":
        cmd = ["conda", "activate", "spear-env&", "pip", "install", "-e", "python"]
    else:
        cmd = ["conda activate spear-env; pip install -e python"]
    print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

    # change current working directory
    print(f"[SPEAR | build_spear.py] Changing current working dir from {os.getcwd()} to {os.path.join(args.repo_dir, 'tools')}")
    os.chdir(os.path.join(args.repo_dir, "tools"))

    if sys.platform == "win32":
        cmd = ["python", "build_third_party_libs.py", "--num_parallel_jobs", f"{args.num_parallel_jobs}"]
    else:    
        cmd = [f"python build_third_party_libs.py --num_parallel_jobs {args.num_parallel_jobs}"]
    print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

    if sys.platform == "win32":
        cmd = ["conda", "activate", "spear-env&", "python", "create_symbolic_links.py"]
    else:
        cmd = ["conda activate spear-env; python create_symbolic_links.py"]
    print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

    if sys.platform == "win32":
       cmd = ["conda", "activate", "spear-env&", "python", "generate_config.py"]
    else: 
        cmd = ["conda activate spear-env; python generate_config.py"]
    print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

    # build the executable
    if sys.platform == "win32":
        build_exe = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.bat")
    else:
        build_exe = os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh")
    
    cmd = [
        build_exe,
        "BuildCookRun",
        "-project=" + os.path.join(args.repo_dir, "cpp", "unreal_projects", "SpearSim", "SpearSim.uproject"), 
        "-build",
        "-cook",
        "-stage",
        "-package",
        "-archive",
        "-pak",
        "-targetplatform=" + args.target_platform,
        "-target=SpearSim",
        "-archivedirectory=" + args.output_dir,
        "-clientconfig=" + args.config_mode,
        "-clean"
    ]
    print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    # change working directory
    print(f"[SPEAR | build_spear.py] Changing current working dir from {os.getcwd()} to {cwd}")
    os.chdir(cwd)

    print(f"[SPEAR | build_spear.py] Successfully built SpearSim for {args.target_platform} platform with {args.config_mode} mode.")

    # zip the exectuable for distribution
    if sys.platform == "win32":
        file = shutil.make_archive(base_name=os.path.join(args.output_dir, f"SpearSim-{args.tag}-{args.target_platform}-{args.config_mode}"), format='zip', root_dir=os.path.join(args.output_dir, "WindowsNoEditor"))
        shutil.rmtree(os.path.join(args.output_dir, "WindowsNoEditor"))
    elif sys.platform == "linux":
        file = shutil.make_archive(base_name=os.path.join(args.output_dir, f"SpearSim-{args.tag}-{args.target_platform}-{args.config_mode}"), format='zip', root_dir=os.path.join(args.output_dir, "LinuxNoEditor"))
        shutil.rmtree(os.path.join(args.output_dir, "LinuxNoEditor"))
    elif sys.platform == "darwin":
        file = shutil.make_archive(base_name=os.path.join(args.output_dir, f"SpearSim-{args.tag}-{args.target_platform}-{args.config_mode}"), format='zip', root_dir=os.path.join(args.output_dir, "MacNoEditor"))
        shutil.rmtree(os.path.join(args.output_dir, "MacNoEditor"))
    
    print(f"[SPEAR | build_spear.py] Successfully created a zip file at {file}, ready for distribution.")