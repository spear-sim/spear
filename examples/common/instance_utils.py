#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear

def open_level(instance, scene_id, map_id=""):

    def begin_tick(instance):
        instance.engine_service.begin_tick()
        instance.game_world_service.unpause_game()

    def tick(instance):
        instance.engine_service.tick()

    def end_tick(instance):
        instance.game_world_service.pause_game()
        instance.engine_service.end_tick()

    begin_tick(instance)
    current_scene_id = instance.game_world_service.get_current_level()
    tick(instance)
    end_tick(instance)

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

        begin_tick(instance)
        instance.game_world_service.open_level(desired_level_name)
        tick(instance)
        end_tick(instance)

    while current_scene_id != scene_id:
        begin_tick(instance)
        current_scene_id = instance.game_world_service.get_current_level()
        tick(instance)
        end_tick(instance)
