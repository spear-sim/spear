#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import spear.editor
import time
import unreal


delay_seconds = 5.0
post_tick_callback_handle = None
start_time_seconds = time.time()
file = __file__

def post_tick_callback(delta_time):
    global post_tick_callback_handle, start_time_seconds, file

    spear.log("post_tick_callback(delta_time=" + str(delta_time) + ")")

    elapsed_time_seconds = time.time() - start_time_seconds
    if elapsed_time_seconds > delay_seconds:
        spear.log("unreal.unregister_slate_post_tick_callback(...)")
        unreal.unregister_slate_post_tick_callback(post_tick_callback_handle)

        spear.log("unreal.AutomationLibrary.take_high_res_screenshot(...)")
        screenshot = os.path.realpath(os.path.join(os.path.dirname(file), "screenshot.png"))
        unreal.AutomationLibrary.take_high_res_screenshot(1024, 1024, screenshot)

        # send message to outer Python script

        spear.log("unreal.get_default_object(...)")
        sp_message_queue_manager_default_object = unreal.get_default_object(unreal.SpMessageQueueManager)

        spear.log("sp_message_queue_manager_default_object.push_message_to_back_of_queue(...)")
        sp_message_queue_manager_default_object.push_message_to_back_of_queue("take_screenshot", "Screenshot saved to path: " + screenshot)


if __name__ == "__main__":

    spear.log("unreal.register_slate_post_tick_callback(...)")
    post_tick_callback_handle = unreal.register_slate_post_tick_callback(post_tick_callback)
    spear.log("Done.")
