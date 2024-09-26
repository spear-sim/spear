import os
import sys

# We use print(...) here because we don't want to depend on the spear module. This script runs automatically
# when the editor starts, and the user might not have run tools/configure_editor_python_env.py yet, so we
# can't assume the spear module is available.
pipeline_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", "pipeline"))
print("[SPEAR | init_unreal.py:0008] Adding to the Unreal Editor's Python sys.path: " + pipeline_dir)
sys.path.append(pipeline_dir)
