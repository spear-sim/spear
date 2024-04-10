#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

def open_level(instance, scene_id, map_id=""):

    instance.engine_service.begin_tick()
    current_scene_id = instance.game_world_service.get_current_level()

    if current_scene_id != scene_id:
        desired_level_name = ""
        if scene_id != "":
            if map_id == "":
                map_id = scene_id
            else:
                map_id = map_id
            desired_level_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id

        spear.log("scene_id:           ", scene_id)
        spear.log("map_id:             ", map_id)
        spear.log("desired_level_name: ", desired_level_name)

        instance.game_world_service.open_level(desired_level_name)
    instance.engine_service.tick()
    instance.engine_service.end_tick()

    while current_scene_id != scene_id:
        instance.engine_service.begin_tick()
        current_scene_id = instance.game_world_service.get_current_level()
        instance.engine_service.tick()
        instance.engine_service.end_tick()
