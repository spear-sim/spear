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

    spear.log("scene_id:   ", scene_id)
    spear.log("map_id:     ", map_id)
    spear.log("level_name: ", level_name)

    instance.engine_service.begin_tick()
    gameplay_statics_class = instance.game_world_service.get_static_class(class_name="UGameplayStatics")
    assert gameplay_statics_class
    gameplay_statics_default_object = instance.game_world_service.get_default_object(uclass=gameplay_statics_class, create_if_needed=False)
    assert gameplay_statics_default_object
    open_level_func = instance.game_world_service.find_function_by_name(uclass=gameplay_statics_class, name="OpenLevel", include_super_flag="IncludeSuper")
    assert open_level_func
    instance.game_world_service.call_function(uobject=gameplay_statics_default_object, ufunction=open_level_func, args={"LevelName": level_name})
    instance.engine_service.tick()
    instance.engine_service.end_tick()

    instance.engine_service.begin_tick()
    current_scene_id = instance.game_world_service.get_world_name()
    instance.engine_service.tick()
    instance.engine_service.end_tick()

    assert current_scene_id == scene_id
