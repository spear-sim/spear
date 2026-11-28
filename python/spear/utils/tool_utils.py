#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import glob
import ntpath
import os
import posixpath
import shlex
import shutil
import spear
import subprocess
import sys

#
# This comment describes how our tools handle paths and quotes, including paths and argument values that
# contain spaces, on Windows, macOS, and Linux. The first section gives recommendations for users of our
# command-line tools. The second section gives recommendations for Python code that launches subprocesses.
#
# RECOMMENDATIONS FOR COMMAND-LINE TOOL USERS
#
# 1. Arguments that are defined by our tools (e.g., --unreal-engine-dir, --unreal-project-dir, --script,
#    --config-files). If a path contains spaces, enclose it in quotes using your shell's normal quoting rules.
#    On Windows, paths can use backslashes or forward slashes.
#
#        Windows Developer PowerShell for VS 2022:  python run_editor_script.py --unreal-engine-dir "C:\my dir\UE" --script "C:\my dir\script.py"
#        Windows Command Prompt:                    python run_editor_script.py --unreal-engine-dir "C:\my dir\UE" --script "C:\my dir\script.py"
#        macOS and Linux:                           python run_editor_script.py --unreal-engine-dir "/my dir/UE" --script "/my dir/script.py"
#
# 2. Extra arguments that our tools forward to the Unreal Editor or a standalone executable (e.g., unknown
#    arguments passed to run_editor_mrq_job.py). If an argument contains spaces, enclose it in quotes using
#    your shell's normal quoting rules. Use forward slashes in paths.
#
#        Windows Developer PowerShell for VS 2022:  python run_editor_mrq_job.py ... "-ExecCmds=r.MyCVar 1" "-MyPath=C:/my dir/my file.txt"
#        Windows Command Prompt:                    python run_editor_mrq_job.py ... "-ExecCmds=r.MyCVar 1" "-MyPath=C:/my dir/my file.txt"
#        macOS and Linux:                           python run_editor_mrq_job.py ... "-ExecCmds=r.MyCVar 1" "-MyPath=/my dir/my file.txt"
#
#    Values in SPEAR.LAUNCHER.COMMAND_LINE_ARGS don't pass through a shell, so don't enclose them in quotes,
#    even if they contain spaces.
#
# 3. Extra arguments that our tools forward to RunUAT or UnrealBuildTool (e.g., unknown arguments passed to
#    run_uat.py or run_build.py). If an argument contains spaces, enclose it in quotes using your shell's
#    normal quoting rules. Use forward slashes in paths.
#
#        Windows Developer PowerShell for VS 2022:  python run_uat.py ... "-cookdir=C:/my dir/Content/MyContent"
#        Windows Command Prompt:                    python run_uat.py ... "-cookdir=C:/my dir/Content/MyContent"
#        macOS and Linux:                           python run_uat.py ... "-cookdir=/my dir/Content/MyContent"
#
# 4. Extra arguments that our tools forward to a Python script running inside the Unreal Editor (e.g.,
#    unknown arguments passed to run_editor_script.py). If an argument contains spaces, enclose it in quotes
#    using your shell's normal quoting rules. On Windows, paths can use backslashes or forward slashes.
#
#        Windows Developer PowerShell for VS 2022:  python run_editor_script.py ... --script "C:\my dir\script.py" --my-path "C:\my dir\my file.txt"
#        Windows Command Prompt:                    python run_editor_script.py ... --script "C:\my dir\script.py" --my-path "C:\my dir\my file.txt"
#        macOS and Linux:                           python run_editor_script.py ... --script "/my dir/script.py" --my-path "/my dir/my file.txt"
#
# 5. Python scripts that you run from the Unreal Editor's console. In "Cmd" mode, type py followed by the
#    script path and its arguments. In "Python" mode, type the script path and its arguments without py. In
#    both modes, the Unreal Editor parses the line itself rather than passing it through a shell. If the script
#    path or an argument contains spaces, enclose it in double quotes. For arguments of the form --name=value,
#    enclose the entire argument in quotes (e.g., "--my-path=/my dir/my file.txt") or pass the name and value
#    separately (e.g., --my-path "/my dir/my file.txt"), because quotes that begin in the middle of an argument
#    are passed to your script. Paths can use backslashes or forward slashes.
#
#        Cmd mode on Windows:                       py "C:\my dir\script.py" --my-path "C:\my dir\my file.txt"
#        Cmd mode on macOS and Linux:               py "/my dir/script.py" --my-path "/my dir/my file.txt"
#        Python mode on Windows:                    "C:\my dir\script.py" --my-path "C:\my dir\my file.txt"
#        Python mode on macOS and Linux:            "/my dir/script.py" --my-path "/my dir/my file.txt"
#
# The following arguments are unsupported:
#
#     - On Windows, quoted paths that end with a backslash, e.g., "C:\my dir\", because the backslash escapes
#       the closing quote. Omit the trailing backslash, e.g., "C:\my dir".
#     - Double quotes and empty arguments in arguments that are forwarded to a Python script running inside
#       the Unreal Editor (recommendations 4 and 5). This is a limitation of the Unreal Editor, not our tools.
#       The Unreal Editor splits the string inside -script="..." or -ExecutePythonScript="...", or the text
#       after the script path in the console, into sys.argv using FParse::Token(...), which has no way to
#       escape a double quote inside an argument, and which stops at the first empty argument. See:
#           Engine/Plugins/Experimental/PythonScriptPlugin/Source/PythonScriptPlugin/Private/PythonScriptPlugin.cpp
#           Engine/Source/Runtime/Core/Private/Misc/Parse.cpp.
#     - Script paths that contain ".py" before the end of the file name (e.g., "/my.python/script.py") when
#       running a script from the Unreal Editor's console (recommendation 5), because the Unreal Editor treats
#       everything up to the first ".py" as the script path. If the Unreal Editor can't find a valid script
#       path, it executes the entire line as Python code instead.
#     - In "Cmd" mode in the Unreal Editor's console (recommendation 5), the characters | and // outside of
#       quotes, because the Unreal Editor interprets | as a separator between multiple commands, and // as the
#       start of a comment. Enclose any argument that contains these characters in quotes.
#     - Tabs and newlines in arguments that are forwarded to the Unreal Editor or a standalone executable
#       (recommendation 2), and arguments that contain a space and an "=" but don't begin with "-" (e.g., a
#       positional argument like "my dir/a=b"), because Unreal on macOS splits these arguments at the first
#       "=" when it adds quotes around them (see convention 3 in our INTERNAL CONVENTIONS below).
#     - On Windows, the characters %, ^, &, |, <, >, and ! in any path or argument that is passed to RunUAT or
#       UnrealBuildTool (recommendation 3, and paths passed to run_uat.py, run_build.py, build_executable.py,
#       and run_generate_project_files.py), because these arguments pass through a .bat file, which interprets
#       these characters.
#
# RECOMMENDATIONS FOR PYTHON CALLERS
#
# Always launch subprocesses using the helper functions in this file, rather than calling subprocess
# directly, and always pass a list of arguments without any added quotes. Each helper function logs a
# command string that can be pasted into a shell to launch the program with identical arguments. On Windows,
# the logged command string is formatted for Windows Developer PowerShell for VS 2022 by default, and can be
# formatted for Windows Command Prompt instead by calling
# spear.tools.set_windows_log_format(windows_log_format="command_prompt"). On macOS and Linux, the logged
# command string is formatted for bash or zsh.
#
# 1. Non-Unreal programs (e.g., b2, cmake, codesign, dsymutil, git, pip, python, signtool). Use
#    spear.tools.run_cmd(...) or spear.tools.popen_cmd(...).
#
#        cmd = ["dsymutil", so_file, "-o", f"{so_file}.dSYM"]
#        spear.tools.run_cmd(cmd=cmd)
#
# 2. Programs in a conda environment. Use spear.tools.run_conda_cmd(...).
#
#        cmd = ["python", script, "--input-file", input_file]
#        spear.tools.run_conda_cmd(conda_env=conda_env, cmd=cmd)
#
# 3. Unreal C++ programs (e.g., UnrealEditor, UnrealPak, or a standalone executable). Use
#    spear.tools.run_unreal_cmd(...) or spear.tools.popen_unreal_cmd(...), and use
#    spear.tools.normalize_unreal_cmd_path(...) to convert every path that we pass to the program to use
#    forward slashes. The helper functions for launching the program can't do this conversion for us, because
#    they don't know which arguments are paths. To pass arguments to a Python script running inside the
#    Unreal Editor, use spear.tools.get_unreal_python_script_args_string(...) to build the value of
#    -script=... or -ExecutePythonScript=....
#
#        script_args = [
#            spear.tools.normalize_unreal_cmd_path(path=script),
#            "--my-arg", "1"]
#        script_args_string = spear.tools.get_unreal_python_script_args_string(args=script_args)
#
#        cmd = [
#            unreal_editor_bin,
#            spear.tools.normalize_unreal_cmd_path(path=uproject),
#            "-run=pythonscript",
#            f"-script={script_args_string}"]
#        spear.tools.run_unreal_cmd(cmd=cmd)
#
# 4. Unreal C# tools invoked through scripts (e.g., RunUAT, Build, GenerateProjectFiles). Use
#    spear.tools.run_uat(...), spear.tools.run_build(...), or spear.tools.run_generate_project_files(...),
#    and use spear.tools.normalize_unreal_cmd_path(...) for paths as in recommendation 3.
#
#        run_uat_args = [
#            "BuildCookRun",
#            f"-project={spear.tools.normalize_unreal_cmd_path(path=uproject)}"]
#        spear.tools.run_uat(unreal_engine_dir=unreal_engine_dir, args=run_uat_args)
#
# For all kinds of programs, use a spear.tools.run_*(...) function if we only need to wait for the program to
# finish. Use a spear.tools.popen_*(...) function if we need to read the program's output while it is
# running (e.g., to parse it), or if we don't want to wait for the program to finish (e.g., spear.Instance).
# If we use a spear.tools.popen_*(...) function and wait for the program to finish, we need to check the
# return code explicitly, unless a non-zero return code is expected (e.g., signtool returns a non-zero return
# code for unsigned files).
#
#        cmd = ["codesign", "--verify", "--deep", "--strict", "--verbose", sign_file]
#        ps = spear.tools.popen_cmd(cmd=cmd, stderr=subprocess.PIPE, text=True)
#        for line in ps.stderr:
#            ...
#        return_code = ps.wait()
#        ps.stderr.close()
#        assert return_code == 0
#

#
# Return a default set of maps to cook. This set is needed in several places, i.e., build_executable.py and
# run_uat.py. We would like to specify directories to always cook in a similar way, but specifying multiple -cookdir
# arguments on the command-line doesn't work reliably. So we specify cook directories in DefaultGame.ini.
#

def get_default_maps_to_cook():
    return [
        "apartment_0000",           # /Game/SPEAR/Scenes/apartment_0000/Maps/apartment_0000
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
# Helper functions for building C++ code outside of the Unreal Build Tool. Used by build_third_party_libs.py and
# install_python_extension.py.
#

def get_macos_deployment_target():
    return "11.0"

# On Linux, we build with Unreal's clang and libc++ so our code is ABI-compatible with Unreal.
def get_unreal_linux_toolchain(unreal_engine_dir):
    clang_path_template = spear.path.realpath(unreal_engine_dir, "Engine", "Extras", "ThirdPartyNotUE", "SDKs", "HostLinux", "Linux_x64", "*clang*")
    clang_paths = glob.glob(clang_path_template)
    assert len(clang_paths) == 1
    clang_path = clang_paths[0]

    spear.log("Found Unreal clang: ", clang_path)

    clang_bin_dir      = spear.path.realpath(clang_path, "x86_64-unknown-linux-gnu", "bin")
    libcpp_include_dir = spear.path.realpath(unreal_engine_dir, "Engine", "Source", "ThirdParty", "Unix", "LibCxx", "include", "c++", "v1")
    libcpp_lib_dir     = spear.path.realpath(unreal_engine_dir, "Engine", "Source", "ThirdParty", "Unix", "LibCxx", "lib", "Unix", "x86_64-unknown-linux-gnu")

    assert os.path.isdir(libcpp_include_dir) and os.listdir(libcpp_include_dir)
    assert os.path.isdir(libcpp_lib_dir) and os.listdir(libcpp_lib_dir)

    spear.log("Found Unreal libc++ include dir: ", libcpp_include_dir)
    spear.log("Found Unreal libc++ lib dir:     ", libcpp_lib_dir)

    # We don't use spear.path.realpath(...) here, because clang++ is typically a symlink to clang, and clang decides
    # whether to compile C or C++ based on the name it is invoked with.
    cxx_compiler = os.path.join(clang_bin_dir, "clang++")

    return {"cxx_compiler": cxx_compiler, "libcpp_include_dir": libcpp_include_dir, "libcpp_lib_dir": libcpp_lib_dir}


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
        # We don't use spear.path.realpath(...) here, because we want to check the ownership of "~/.config/"
        # itself, not the directory it points to if it is a symlink.
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
    run_uat_script = spear.path.realpath(unreal_engine_dir, "Engine", "Build", "BatchFiles", run_uat_script_name)
    assert os.path.exists(run_uat_script)

    run_cmd(cmd=[run_uat_script] + args) # see INTERNAL CONVENTIONS below

def run_build(unreal_engine_dir, args):
    if sys.platform == "win32":
        build_script_suffix = "Build.bat"
    elif sys.platform == "darwin":
        build_script_suffix = os.path.join("Mac", "Build.sh")
    elif sys.platform == "linux":
        build_script_suffix = os.path.join("Linux", "Build.sh")
    else:
        assert False
    build_script = spear.path.realpath(unreal_engine_dir, "Engine", "Build", "BatchFiles", build_script_suffix)
    assert os.path.exists(build_script)

    # UnrealBuildTool returns exit code 2 when every target is already up-to-date, which isn't an error, so we
    # treat it as success on every platform. Mac's Build.sh additionally treats exit code 255 (UnrealBuildTool
    # was canceled, e.g., via Ctrl-C) and exit code 254 (not used by any current UnrealBuildTool code path, so
    # presumably vestigial) as success internally, before we ever see its return code here, but we don't
    # attempt to replicate that choice on Windows or Linux.
    cmd = [build_script] + args
    result = run_cmd(cmd=cmd, check=False) # see INTERNAL CONVENTIONS below
    if result.returncode not in [0, 2]:
        raise subprocess.CalledProcessError(result.returncode, cmd)

def run_generate_project_files(unreal_engine_dir, args):
    if sys.platform == "win32":
        generate_project_files_script = spear.path.realpath(unreal_engine_dir, "Engine", "Build", "BatchFiles", "Build.bat")
        generate_project_files_platform_args = ["-projectfiles"]
    elif sys.platform == "darwin":
        generate_project_files_script = spear.path.realpath(unreal_engine_dir, "Engine", "Build", "BatchFiles", "Mac", "GenerateProjectFiles.sh")
        generate_project_files_platform_args = []
    elif sys.platform == "linux":
        generate_project_files_script = spear.path.realpath(unreal_engine_dir, "Engine", "Build", "BatchFiles", "Linux", "GenerateProjectFiles.sh")
        generate_project_files_platform_args = []
    else:
        assert False
    assert os.path.exists(generate_project_files_script)

    run_cmd(cmd=[generate_project_files_script] + args + generate_project_files_platform_args) # see INTERNAL CONVENTIONS below


#
# Helper functions for launching subprocesses.
#

#
# In this comment, we describe how our helper functions work, including how they log command strings
# depending on set_windows_log_format(...), and explain why they are correct, provided that users and Python
# call sites follow the recommendations at the top of this file.
#
# INTERNAL CONVENTIONS
#
# Each helper function receives a list of arguments without any added quotes. We never use shell=True, for
# two reasons. First, the shell would interpret special characters in paths (e.g., %, ^, and & on Windows).
# Second, the process ID returned by subprocess would refer to the shell, rather than to the program, which
# matters for code that monitors the program (e.g., spear.Instance).
#
# To log a command string that can be pasted into a shell on macOS and Linux, we use shlex.join(...). On
# Windows, a program receives a single command-line string, rather than argv. When we pass a list to
# subprocess, subprocess converts it to a command-line string using subprocess.list2cmdline(...), and for
# Unreal C++ programs, we construct the command-line string ourselves (see convention 3 below). In both
# cases, we log the program path followed by the arguments portion of this command-line string, formatted
# for Windows Developer PowerShell for VS 2022 (the default) or Windows Command Prompt, depending on
# set_windows_log_format(...). Windows Command Prompt passes the arguments portion to the program verbatim.
# In Windows PowerShell, we use the --% token, which causes PowerShell to pass the rest of the line to the
# program verbatim, e.g.,
#
#        Windows Developer PowerShell for VS 2022:  & 'C:\my dir\program.exe' --% "C:/my dir/my file.txt" -my-arg="a b"
#        Windows Command Prompt:                    "C:\my dir\program.exe" "C:/my dir/my file.txt" -my-arg="a b"
#
# 1. Non-Unreal programs. We pass our list to subprocess unmodified. In this case, subprocess handles quoting
#    correctly on all platforms, so the program receives exactly the arguments in our list.
#
# 2. Programs in a conda environment. We prepend "conda run -n <env> --no-capture-output" to our list and
#    then follow convention 1. We need --no-capture-output to see output as it is produced. We use the
#    CONDA_EXE environment variable, rather than "conda", because "conda" refers to a .bat file on Windows.
#
# 3. Unreal C++ programs.
#
#    macOS and Linux: Unreal rebuilds its command-line string from argv, adding quotes around every argument
#    that contains a space. If the argument begins with "-" and contains an "=", then Unreal adds quotes around
#    the value after the first "=", rather than around the entire argument. On macOS, Unreal does this even if
#    the argument doesn't begin with "-", which is why these arguments are unsupported (see
#    Engine/Source/Runtime/Launch/Private/Mac/LaunchMac.cpp and
#    Engine/Source/Runtime/Unix/UnixCommonStartup/Private/UnixCommonStartup.cpp). So we pass our list to
#    subprocess unmodified, e.g.,
#
#        ["/UE/.../UnrealEditor", "/my dir/MyProject.uproject", "-run=pythonscript", "-script=/scripts/script.py --my-arg 1"]
#
#    and Unreal sees the following command-line string:
#
#        "/my dir/MyProject.uproject" -run=pythonscript -script="/scripts/script.py --my-arg 1"
#
#    Windows: Unreal parses the raw command-line string, rather than argv, and expects the quotes for an
#    argument's value to begin after the "=". If we passed a list to subprocess, it would add quotes around
#    the entire argument, and Unreal would truncate the value at the first space, e.g., "-key=C:/a b" would be
#    parsed as C:/a. So we convert our list to a command-line string by adding quotes in the same way that
#    Unreal adds them on macOS and Linux, and pass the string to subprocess, which passes it to Unreal
#    verbatim. So Unreal sees the same command-line string on all platforms, e.g.,
#
#        "C:\UE\...\UnrealEditor-Cmd.exe" "C:/my dir/MyProject.uproject" -run=pythonscript -script="C:/scripts/script.py --my-arg 1"
#
#    Forward slashes: The -script="..." and -ExecutePythonScript="..." arguments are parsed with
#    FParse::QuotedString(...), which interprets backslash escape sequences. Other arguments are parsed with
#    FParse::Value(...), which doesn't, but we use forward slashes everywhere for simplicity. Unreal accepts
#    forward slashes on all platforms.
#
#    Extra arguments for Unreal from the user (recommendation 2 for users of our command-line tools) are
#    appended to our list, so they are quoted in exactly the same way as our own arguments.
#
#    Arguments for a Python script running inside the Unreal Editor, including extra arguments from the user
#    (recommendation 4 for users of our command-line tools), are converted to the value of -script=... or
#    -ExecutePythonScript=... using get_unreal_python_script_args_string(...). The Unreal Editor parses this
#    value with FParse::QuotedString(...), which interprets backslash escape sequences, and then splits the
#    result into sys.argv with FParse::Token(...), which removes quotes around arguments that contain spaces.
#    So we replace each backslash with \\, and add \" around each argument that contains whitespace, e.g.,
#
#        Arguments to run_editor_script.py:  --script "C:\my dir\script.py" --my-path "C:\my dir\my file.txt"
#        Command-line string for Unreal:     -script="\"C:/my dir/script.py\" --my-path \"C:\\my dir\\my file.txt\""
#        sys.argv in the inner script:       ["C:/my dir/script.py", "--my-path", "C:\my dir\my file.txt"]
#
#    The value only contains escape sequences if it contains more than one argument, because the script path
#    always uses forward slashes, so the value always contains a space in this case, and Unreal (or our helper
#    function on Windows) always adds the quotes that FParse::QuotedString(...) expects.
#
# 4. Unreal C# tools invoked through scripts. We follow convention 1. These scripts forward their arguments
#    unmodified (via %* in RunUAT.bat and Build.bat, and via "$@" in RunUAT.sh, Build.sh, and
#    GenerateProjectFiles.sh), and the C# tools parse their arguments using standard argv rules, so the C#
#    tools receive exactly the arguments in our list. We use forward slashes because these tools sometimes
#    pass paths through to Unreal C++ programs (e.g., RunUAT passes -cookdir to UnrealEditor when cooking).
#
#    Extra arguments for RunUAT or UnrealBuildTool from the user (recommendation 3 for users of our
#    command-line tools) are appended to our list, so the C# tools receive exactly the arguments that the
#    user's shell passed to our tool. On Windows, cmd.exe interprets special characters while running the .bat
#    file, which is why these characters are unsupported.
#

_windows_log_format = "powershell"

def set_windows_log_format(windows_log_format):
    global _windows_log_format
    assert windows_log_format in ["powershell", "command_prompt"]
    _windows_log_format = windows_log_format

def run_cmd(cmd, check=True, **kwargs):
    spear.log("Executing: ", _get_cmd_string(cmd=cmd))
    return subprocess.run(cmd, check=check, **kwargs)

def popen_cmd(cmd, **kwargs):
    spear.log("Executing: ", _get_cmd_string(cmd=cmd))
    return subprocess.Popen(cmd, **kwargs)

def run_conda_cmd(conda_env, cmd, check=True, **kwargs):
    # "conda" refers to a .bat file on Windows, so we use CONDA_EXE instead
    conda_bin = os.environ["CONDA_EXE"]
    assert os.path.exists(conda_bin)
    return run_cmd(cmd=[conda_bin, "run", "-n", conda_env, "--no-capture-output"] + cmd, check=check, **kwargs)

def run_unreal_cmd(cmd, check=True, **kwargs):
    unreal_cmd = _get_unreal_cmd(cmd=cmd)
    spear.log("Executing: ", _get_cmd_string(cmd=cmd, unreal=True))
    return subprocess.run(unreal_cmd, check=check, **kwargs)

def popen_unreal_cmd(cmd, **kwargs):
    unreal_cmd = _get_unreal_cmd(cmd=cmd)
    spear.log("Executing: ", _get_cmd_string(cmd=cmd, unreal=True))
    return subprocess.Popen(unreal_cmd, **kwargs)

def normalize_unreal_cmd_path(path):
    return path.replace(ntpath.sep, posixpath.sep)

def _get_cmd_string(cmd, unreal=False):
    if sys.platform == "win32":
        if unreal:
            args_string = _get_unreal_args_string(args=cmd[1:])
        else:
            args_string = subprocess.list2cmdline(cmd[1:])

        if _windows_log_format == "powershell":
            program_string = "& '" + cmd[0].replace("'", "''") + "'"
            args_string = "--% " + args_string
        elif _windows_log_format == "command_prompt":
            program_string = subprocess.list2cmdline([cmd[0]])
        else:
            assert False

        if len(cmd) == 1:
            return program_string
        else:
            return f"{program_string} {args_string}"

    elif sys.platform in ["darwin", "linux"]:
        return shlex.join(cmd)
    else:
        assert False

def _get_unreal_cmd(cmd):
    for arg in cmd[1:]:
        assert "\t" not in arg
        assert "\n" not in arg
        # On macOS, Unreal splits an argument that contains a space at its first "=", even if the argument
        # doesn't begin with "-", so we don't support arguments like this that contain an "="
        if not arg.startswith("-") and " " in arg:
            assert "=" not in arg

    if sys.platform == "win32":
        return subprocess.list2cmdline([cmd[0]]) + " " + _get_unreal_args_string(args=cmd[1:])
    elif sys.platform in ["darwin", "linux"]:
        return cmd
    else:
        assert False

def _get_unreal_args_string(args):
    # Unreal parses the raw command-line string on Windows, so we add quotes the same way that Unreal adds them
    # on macOS and Linux when it reconstructs the command-line string from argv.
    arg_strings = []
    for arg in args:
        if " " in arg and arg.startswith("-") and "=" in arg:
            arg_name, arg_value = arg.split("=", 1)
            arg_strings.append(f'{arg_name}="{arg_value}"')
        elif " " in arg:
            arg_strings.append(f'"{arg}"')
        else:
            arg_strings.append(arg)
    return " ".join(arg_strings)


#
# Helper function for forwarding arguments to a Python script running inside the Unreal Editor via
# -script=... or -ExecutePythonScript=.... See convention 3 in INTERNAL CONVENTIONS above.
#

def get_unreal_python_script_args_string(args):
    arg_strings = []
    for arg in args:
        # the Unreal Editor can't pass empty arguments or arguments that contain double quotes to a Python script
        assert arg != ""
        assert '"' not in arg
        arg_string = arg.replace("\\", "\\\\")
        if any(c.isspace() for c in arg):
            arg_string = f'\\"{arg_string}\\"'
        arg_strings.append(arg_string)
    return " ".join(arg_strings)

