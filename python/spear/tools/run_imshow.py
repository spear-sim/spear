#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import cv2
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--path", required=True)
parser.add_argument("--name", default="spear.imshow()")
args = parser.parse_args()

image = cv2.imread(args.path, cv2.IMREAD_UNCHANGED)
if image is None:
    print(f"Failed to load image: {args.path}", file=sys.stderr)
    sys.exit(1)

cv2.imshow(args.name, image)
cv2.waitKey(0)
cv2.destroyAllWindows()
