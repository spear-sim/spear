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

    # call OpenLevel(...) at the beginning of a frame
    instance.engine_service.begin_tick()

    gameplay_statics_class = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
    gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_class)
    open_level_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_class, function_name="OpenLevel")
    instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=open_level_func, args={"LevelName": level_name})

    instance.engine_service.tick()
    instance.engine_service.end_tick()

    # on the next frame, make sure that we've loaded the desired level
    instance.engine_service.begin_tick()
    current_scene_id = instance.unreal_service.get_world_name()
    instance.engine_service.tick()
    instance.engine_service.end_tick()

    assert current_scene_id == scene_id
