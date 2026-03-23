#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import spear
import time
import unreal


delay_seconds = 5.0
file = __file__


@spear.editor.script
def script():
    start_time_seconds = time.time()

    while time.time() - start_time_seconds < delay_seconds:
        spear.log(f"waiting... delta_time={spear.editor.get_script_delta_time()}")
        yield

    spear.log("unreal.AutomationLibrary.take_high_res_screenshot(...)")
    screenshot = os.path.realpath(os.path.join(os.path.dirname(file), "screenshot.png"))
    spear.log("Saving screenshot: ", screenshot)
    unreal.AutomationLibrary.take_high_res_screenshot(res_x=1024, res_y=1024, filename=screenshot)

    spear.log("Done.")


if __name__ == "__main__":
    script()
