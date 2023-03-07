#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import subprocess

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--executable_dir", required=True)
    parser.add_argument("--output_dir", default=os.path.join(os.))
    args = parser.parse_args()

    if args.platform == "win32":
        file = shutil.make_archive(base_name=os.path.join(args.output_dir, f"SpearSim-{args.output_tag}-{args.target_platform}-{args.config_mode}"), format='zip', root_dir=os.path.join(args.temp_dir, f"SpearSim-{args.config_mode}", platform_dir_name))
    elif args.platform == "darwin":
        notarized_zip = os.path.join(args.output_dir, f"{os.path.splitext(os.path.basename(executable))[0]}-notarized.zip")
        cmd = ["ditto", "-c", "-k", "--rsrc", "--keepParent", os.path.dirname(executable), notarized_zip]
        print(f"[SPEAR | sign_executable.py] Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
    elif args.platform == "linux":
        file = shutil.make_archive(base_name=os.path.join(args.output_dir, f"SpearSim-{args.output_tag}-{args.target_platform}-{args.config_mode}"), format='zip', root_dir=os.path.join(args.temp_dir, f"SpearSim-{args.config_mode}", platform_dir_name))
    else:
        assert False
