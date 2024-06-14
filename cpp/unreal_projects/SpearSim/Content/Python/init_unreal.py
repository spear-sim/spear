import os
import sys
pipeline_unreal_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", "pipeline", "unreal"))
print("[SPEAR | init_python.py:0004] Adding to the Unreal Editor's Python sys.path: " + pipeline_unreal_dir) # don't want to depend on any spear functionality
sys.path.append(pipeline_unreal_dir)
