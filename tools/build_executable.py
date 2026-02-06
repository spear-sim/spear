#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import spear
import subprocess
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--build-config", required=True)
parser.add_argument("--unreal-engine-dir", required=True)
parser.add_argument("--skip-cook-default-maps", action="store_true")
parser.add_argument("--cook-dirs", nargs="*")
parser.add_argument("--cook-maps", nargs="*")
parser.add_argument("--build-dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "BUILD")))
args, unknown_args = parser.parse_known_args() # get unknown args to pass to RunUAT

assert args.build_config in ["Debug", "DebugGame", "Development", "Shipping", "Test"]
assert os.path.exists(args.unreal_engine_dir)


if __name__ == "__main__":

    repo_dir           = os.path.realpath(os.path.join(args.build_dir, "spear"))
    unreal_project_dir = os.path.realpath(os.path.join(repo_dir, "cpp", "unreal_projects", "SpearSim"))
    uproject           = os.path.realpath(os.path.join(unreal_project_dir, "SpearSim.uproject"))

    # set various platform-specific variables that we use throughout our build procedure
    if sys.platform == "win32":
        target_platform       = "Win64"
        run_uat_script        = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.bat"))
        archive_dir           = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-{target_platform}-{args.build_config}-Unsigned"))
        run_uat_platform_args = ""
        unreal_tmp_dir        = ""

        cxx_compiler = "cl"
        cxx_compiler_path = shutil.which(cxx_compiler)
        if cxx_compiler_path is None:
            spear.log("ERROR: Can't find the Visual Studio command-line tools. All SPEAR build steps must run in a terminal where the Visual Studio command-line tools are visible. Giving up...")
            assert False
        if cxx_compiler_path.lower().endswith("hostx86\\x86\\cl.exe") or cxx_compiler_path.lower().endswith("hostx86\\x64\\cl.exe"):
            spear.log("ERROR: 32-bit terminal detected. All SPEAR build steps must run in a 64-bit terminal. Giving up...")
            spear.log("ERROR: Compiler path:", cxx_compiler_path)
            assert False

    elif sys.platform == "darwin":
        target_platform       = "Mac"
        run_uat_script        = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh"))
        archive_dir           = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-{target_platform}-{args.build_config}-Unsigned"))
        run_uat_platform_args = "-specifiedarchitecture=arm64+x86_64"
        unreal_tmp_dir        = os.path.expanduser(os.path.join("~", "Library", "Preferences", "Unreal Engine", "SpearSimEditor"))

    elif sys.platform == "linux":
        target_platform = "Linux"
        run_uat_script        = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh"))        
        archive_dir           = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-{target_platform}-{args.build_config}"))
        run_uat_platform_args = ""
        unreal_tmp_dir        = ""

    else:
        assert False

    assert os.path.exists(run_uat_script)

    if os.path.exists(archive_dir):
        spear.log("Archive directory exists, removing: ", archive_dir)
        shutil.rmtree(archive_dir, ignore_errors=True)

    # We need to remove this temp dir (created by the Unreal build process) because it contains paths from previous builds.
    # If we don't do this step, we will get many warnings during this build:
    #     Warning: Unable to generate long package name for path/to/previous/build/Some.uasset because FilenameToLongPackageName failed to convert
    #     'path/to/previous/build/Some.uasset'. Attempt result was '../../../../../../path/to/previous/build/path/to/previous/build/Some', but the
    #     path contains illegal characters '.'

    if os.path.exists(unreal_tmp_dir):
        spear.log("Unreal Engine cache directory exists, removing: ", unreal_tmp_dir)
        shutil.rmtree(unreal_tmp_dir, ignore_errors=True)

    # Remove signatures from BootstrapPackagedGame-Win64-Shipping.exe because otherwise we won't be able to sign our own executable:
    #    https://forums.unrealengine.com/t/why-does-windows-signtool-report-packaged-exe-as-invalid/367924/10

    if sys.platform == "win32":

        bootstrap_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "BootstrapPackagedGame-Win64-Shipping.exe"))
        bootstrap_backup_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "BootstrapPackagedGame-Win64-Shipping.backup.exe"))

        spear.log("Determining if file has any signatures: ", bootstrap_bin)

        # verify
        cmd = ["signtool", "verify", "/a", "/pa", "/all", "/v", bootstrap_bin]
        spear.log("Executing: ", " ".join(cmd))
        ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)

        num_signatures = None
        for line in ps.stdout:
            spear.log_no_prefix(line)
            if num_signatures is None and "Number of signatures successfully Verified: " in line:
                num_signatures = int(line.split("Number of signatures successfully Verified: ")[1].strip())
        ps.wait()
        ps.stdout.close()
        assert num_signatures is not None

        if num_signatures > 0:
            spear.log("File already has one ore more signatures, creating backup: ", bootstrap_backup_bin)
            shutil.copy(bootstrap_bin, bootstrap_backup_bin)

            spear.log("Removing all signatures from file: ", bootstrap_bin)
            cmd = ["signtool", "remove", "/s", "/v", bootstrap_bin]
            spear.log("Executing: ", " ".join(cmd))
            subprocess.run(cmd, check=True)
            spear.log("Removed all signatures, proceeding...")
        else:
            spear.log("File has no signatures, proceeding...")

    # assemble dirs to cook

    cook_dirs = []
    if args.cook_dirs is not None:
        cook_dirs = args.cook_dirs

    cook_dir_args = [ f'-cookdir="{os.path.join(unreal_project_dir, cook_dir)}"' for cook_dir in cook_dirs ]

    # assemble maps to cook

    cook_maps = []
    if not args.skip_cook_default_maps:
        cook_maps.extend(spear.utils.tool_utils.get_default_maps_to_cook())
    if args.cook_maps is not None:
        cook_maps.extend(args.cook_maps)

    if len(cook_maps) == 0:
        cook_maps_arg = []
    else:
        cook_maps_arg = [f"-map={'+'.join(cook_maps)}"]

    # build SpearSim project

    cmd = [
        run_uat_script,
        "BuildCookRun",
        f'-project="{uproject}"',
        "-target=SpearSim",
        "-build",   # build C++ executable
        "-cook",    # prepare cross-platform content for a specific platform (e.g., compile shaders)
        "-stage",   # copy C++ executable to a staging directory (required for -pak)
        "-package", # prepare staged executable for distribution on a specific platform (e.g., runs otool and xcrun on macOS)
        "-archive", # copy staged executable to -archivedirectory (on some platforms this will also move directories around relative to the executable)
        "-pak",     # generate a pak file for cooked content and configure executable so it can load pak files
        f"-targetplatform={target_platform}",
        f"-clientconfig={args.build_config}",
        f'-archivedirectory="{archive_dir}"',
        run_uat_platform_args] + \
        unknown_args + \
        cook_dir_args + \
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
