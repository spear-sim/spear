#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import shutil

cxx_compiler = "cl"
cxx_compiler_path = shutil.which(cxx_compiler)
if cxx_compiler_path is None:
    print("[SPEAR | check_terminal_windows.py] ERROR: Can't find the Visual Studio command-line tools. All SPEAR build steps must run in a terminal where the Visual Studio command-line tools are visible. Giving up...")
    assert False
if cxx_compiler_path.lower().endswith("hostx86\\x86\\cl.exe") or cxx_compiler_path.lower().endswith("hostx86\\x64\\cl.exe"):
    print("[SPEAR | check_terminal_windows.py] ERROR: 32-bit terminal detected. All SPEAR build steps must run in a 64-bit terminal. Giving up...")
    print("[SPEAR | check_terminal_windows.py] ERROR: Compiler path:", cxx_compiler_path)
    assert False

print("[SPEAR | check_terminal_windows.py] Terminal is correctly configured.")
