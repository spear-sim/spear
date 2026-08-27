#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import shutil
import spear
import subprocess
import sys

#
# Return a default set of maps to cook. This set is needed in several places, e.g., build_executable.py,
# build_paks.py, and run_uat.py. We would like to specify directories to always cook in a similar way, but
# specifying multiple -cookdir arguments on the command-line doesn't work reliably. So we specify cook
# directories in DefaultGame.ini.
#

def get_default_maps_to_cook():
    return [
        "apartment_0000",           # /Game/SPEAR/Scenes/apartment_0000/Maps/apartment_0000
        "debug_0000",               # /Game/SPEAR/Scenes/debug_0000/Maps/debug_0000
        "debug_0001",               # /Game/SPEAR/Scenes/debug_0001/Maps/debug_0001
        "Advanced_Lighting",        # /Game/StarterContent/Maps/Advanced_Lighting
        "Minimal_Default",          # /Game/StarterContent/Maps/Minimal_Default
        "StarterMap",               # /Game/StarterContent/Maps/StarterMap
        "ThirdPersonMap",           # /Game/ThirdPerson/Maps/ThirdPersonMap
        "VehicleExampleMap",        # /Game/VehicleTemplate/Maps/VehicleExampleMap
        "VehicleOffroadExampleMap"] # /Game/VehicleTemplate/Maps/VehicleOffroadExampleMap


#
# Returns the target platform for use in Unreal command-line tools.
#

def get_target_platform():
    if sys.platform == "win32":
        return "Win64"
    elif sys.platform == "darwin":
        return "Mac"
    elif sys.platform == "linux":
        return "Linux"
    else:
        assert False


#
# Helper functions for building via Build, RunUAT, and UnrealBuildTool. Used by run_build.py, run_uat.py, and build_executable.py.
#

def validate_build_environment():
    if sys.platform == "win32":
        cxx_compiler_path = shutil.which("cl")
        if cxx_compiler_path is None:
            spear.log("ERROR: Can't find the Visual Studio command-line tools. All SPEAR build steps must run in a terminal where the Visual Studio command-line tools are visible. Giving up...")
            assert False
        if cxx_compiler_path.lower().endswith("hostx86\\x86\\cl.exe") or cxx_compiler_path.lower().endswith("hostx86\\x64\\cl.exe"):
            spear.log("ERROR: 32-bit terminal detected. All SPEAR build steps must run in a 64-bit terminal. Giving up...")
            spear.log("ERROR: Compiler path:", cxx_compiler_path)
            assert False

    elif sys.platform in ["darwin", "linux"]:
        # The Unreal Build Tool expects "~/.config/" to be owned by the user, so it can create and write to
        # "~/.config/Unreal Engine/" without requiring admin privileges. This check might seem esoteric, but
        # we have seen cases where "~/.config/" is owned by root in some corporate environments, so we choose
        # to check it here as a courtesy to new users. We don't know if we need a similar check on Windows.
        config_dir = os.path.expanduser(os.path.join("~", ".config"))
        if os.path.exists(config_dir):
            import pwd # not available on Windows
            current_user = pwd.getpwuid(os.getuid()).pw_name
            config_dir_owner = pwd.getpwuid(os.stat(config_dir).st_uid).pw_name
            if current_user != config_dir_owner:
                spear.log(f"ERROR: The Unreal Build Tool expects {current_user} to be the owner of {config_dir}, but the current owner is {config_dir_owner}. To update, run the following command:")
                spear.log(f"    sudo chown {current_user} {config_dir}")
                assert False

    else:
        assert False

def run_uat(unreal_engine_dir, args):
    if sys.platform == "win32":
        run_uat_script_name = "RunUAT.bat"
    elif sys.platform in ["darwin", "linux"]:
        run_uat_script_name = "RunUAT.sh"
    else:
        assert False
    run_uat_script = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Build", "BatchFiles", run_uat_script_name))
    assert os.path.exists(run_uat_script)

    cmd = [run_uat_script] + args
    spear.log("Executing: ", " ".join(cmd))
    subprocess.run(cmd, check=True)

def run_build(unreal_engine_dir, args):
    if sys.platform == "win32":
        build_script_suffix = "Build.bat"
    elif sys.platform == "darwin":
        build_script_suffix = os.path.join("Mac", "Build.sh")
    elif sys.platform == "linux":
        build_script_suffix = os.path.join("Linux", "Build.sh")
    else:
        assert False
    build_script = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Build", "BatchFiles", build_script_suffix))
    assert os.path.exists(build_script)

    cmd = [build_script] + args
    spear.log("Executing: ", " ".join(cmd))

    # UnrealBuildTool returns exit code 2 when every target is already up-to-date, which isn't an error, so we
    # treat it as success on every platform. Mac's Build.sh additionally treats exit code 255 (UnrealBuildTool
    # was canceled, e.g., via Ctrl-C) and exit code 254 (not used by any current UnrealBuildTool code path, so
    # presumably vestigial) as success internally, before we ever see its return code here, but we don't
    # attempt to replicate that choice on Windows or Linux.
    result = subprocess.run(cmd)
    if result.returncode not in [0, 2]:
        raise subprocess.CalledProcessError(result.returncode, cmd)


#
# Helper functions for paths that handle symlinks robustly.
#

def path_exists(path):
    if os.path.exists(path) or os.path.islink(path):
        return True

    head, tail = os.path.split(path)
    if os.path.exists(head) or os.path.islink(head):
        if tail == "":
            return os.path.exists(head) or os.path.islink(head)
        else:
            return tail in os.listdir(head)
    else:
        return False

def remove_path(path):
    if not path_exists(path=path):
        return

    if os.path.islink(path):
        os.unlink(path)
    elif os.path.isfile(path):
        os.remove(path)
    elif os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)
    else:
        # if we have a broken symlink, then try to use the command-line (we don't attempt to use
        # subprocess.run() because it returns an error when attempting to remove broken symlinks
        if sys.platform == "win32":
            rm_cmd = "del"
        elif sys.platform in ["darwin", "linux"]:
            rm_cmd = "rm"
        else:
            assert False
        cmd = f'{rm_cmd} "{path}"'
        cmd_result = os.system(cmd)
        assert cmd_result == 0
