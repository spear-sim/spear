#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--scale", type=float, default=10.0)
args = parser.parse_args()

inner_script_result = np.array([1.0, 2.0, 3.0])*args.scale
