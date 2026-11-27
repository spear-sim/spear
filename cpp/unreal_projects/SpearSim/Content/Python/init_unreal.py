#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import sys

# We use print(...) here because we don't want to depend on the spear module. This script runs automatically
# when the editor starts, and the user might not have run tools/configure_editor_python_env.py yet, so we
# can't assume the spear module is available. For the same reason, we use os.path.realpath(os.path.join(...))
# here instead of spear.path.realpath(...).
editor_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", "editor"))
print("[SPEAR | init_unreal.py] Adding to the Unreal Editor's Python sys.path: " + editor_dir)
sys.path.append(editor_dir)
