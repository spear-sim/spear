#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import numpy as np
import spear
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--num-yields", type=int, default=5)
parser.add_argument("--scale", type=float, default=2.0)
args = parser.parse_args()


@spear.editor.script
def script():
    for i in range(args.num_yields):
        spear.log(f"inner_script_across_frames: yield {i + 1}/{args.num_yields}")
        yield

    global across_frames_result
    across_frames_result = np.array([1.0, 2.0, 3.0])*args.scale


if __name__ == "__main__":
    script()
