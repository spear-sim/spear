#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
# import conda.cli.python_api as Conda
import os
import subprocess
import sys

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--repo_dir", required=True)
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--num_parallel_jobs", default=4)
    parser.add_argument("--target_platform", required=True)
    parser.add_argument("--config_mode", required=True)
    parser.add_argument("--output_dir", required=True)
    args = parser.parse_args()

    os.makedirs(args.repo_dir, exist_ok=True)
    os.makedirs(args.output_dir, exist_ok=True)

    # cmd = ["git", "clone", "--recurse-submodules", "https://github.com/isl-org/spear", args.repo_dir]
    # print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    # cmd_result = subprocess.run(cmd)
    # assert cmd_result.returncode == 0

    cmd = ["conda --version"]
    print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

    # (stdout_str, stderr_str, return_code_int) = Conda.run_command(
    #     Conda.Commands.CREATE,
    #     '-n', 'spear-env', 'python=3.8',
    #     use_exception_handler=True, stdout=sys.stdout, stderr=sys.stderr)
    # assert return_code_int == 0

    cwd = os.getcwd()
    os.chdir(args.repo_dir)

    # (stdout_str, stderr_str, return_code_int) = Conda.run_command(
    #     Conda.Commands.INSTALL, '-n', 'spear-env', use_exception_handler=True, stdout=sys.stdout, stderr=sys.stderr)
    # assert return_code_int == 0

    # cmd = ["conda create -n spear-env python=3.8 -y"]
    # print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    # cmd_result = subprocess.run(cmd, shell=True)
    # assert cmd_result.returncode == 0

    # cmd = ["conda activate spear-env; pip install -e third_party/msgpack-rpc-python"]
    # print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    # cmd_result = subprocess.run(cmd, shell=True)
    # assert cmd_result.returncode == 0

    # cmd = ["conda activate spear-env; pip install -e python"]
    # print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    # cmd_result = subprocess.run(cmd, shell=True)
    # assert cmd_result.returncode == 0

    os.chdir(os.path.join(args.repo_dir, "tools"))

    # cmd = [f"python build_third_party_libs.py --num_parallel_jobs {args.num_parallel_jobs}"]
    # print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    # cmd_result = subprocess.run(cmd, shell=True)
    # assert cmd_result.returncode == 0

    # cmd = ["conda activate spear-env; python create_symbolic_links.py"]
    # print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    # cmd_result = subprocess.run(cmd, shell=True)
    # assert cmd_result.returncode == 0

    cmd = ["conda activate spear-env; python generate_config.py"]
    print(f"[SPEAR | build_spear.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd, shell=True)
    assert cmd_result.returncode == 0

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