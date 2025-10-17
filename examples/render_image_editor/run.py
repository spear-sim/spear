#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import spear
import unreal


if __name__ == "__main__":

    unreal.AutomationLibrary.take_high_res_screenshot(res_x=1024, res_y=1024, filename=os.path.realpath(os.path.join(os.path.dirname(__file__), "screenshot.png")))
    spear.log("Done.")
