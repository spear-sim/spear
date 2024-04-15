#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

def open_level(instance, scene_id, map_id=""):

    level_name = ""
    if scene_id != "":
        if map_id == "":
            map_id = scene_id
        else:
            map_id = map_id
        level_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id

    spear.log("scene_id:           ", scene_id)
    spear.log("map_id:             ", map_id)
    spear.log("level_name: ", level_name)

    instance.engine_service.begin_tick()
    current_scene_id = instance.game_world_service.get_world_name()
    instance.game_world_service.open_level(level_name)
    instance.engine_service.tick()
    instance.engine_service.end_tick()

    while current_scene_id != scene_id:
        instance.engine_service.begin_tick()
        current_scene_id = instance.game_world_service.get_world_name()
        instance.engine_service.tick()
        instance.engine_service.end_tick()
