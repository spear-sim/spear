#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import mayavi.mlab
import numpy as np
import os
import spear
import unreal_geometry_utils


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
parser.add_argument("--visual-parity-with-unreal", action="store_true")
parser.add_argument("--ignore-actors", nargs="*")
parser.add_argument("--color-mode", default="unique_color_per_component")
args = parser.parse_args()

assert os.path.exists(args.pipeline_dir)
assert args.color_mode in ["single_color", "unique_color_per_actor", "unique_color_per_component"]

np.random.seed(0)


def process_scene():

    actors = unreal_geometry_utils.get_actors(pipeline_dir=args.pipeline_dir, ignore_actors=args.ignore_actors)

    unreal_geometry_utils.draw_origin_axes(visual_parity_with_unreal=args.visual_parity_with_unreal)

    color = (0.75, 0.75, 0.75)
    unreal_geometry_utils.draw_actors(pipeline_dir=args.pipeline_dir, actors=actors, color=color, visual_parity_with_unreal=args.visual_parity_with_unreal, color_mode=args.color_mode)

    mayavi.mlab.show()

    spear.log("Done.")


if __name__ == "__main__":
    process_scene()
