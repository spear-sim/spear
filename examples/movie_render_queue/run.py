#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import pprint
import spear
import time


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)

    with instance.begin_frame():

        # get UMoviePipelineQueueEngineSubsystem
        movie_pipeline_queue_engine_subsystem = instance.unreal_service.get_engine_subsystem_by_type(class_name="UMoviePipelineQueueEngineSubsystem")
        spear.log("movie_pipeline_queue_engine_subsystem: ", movie_pipeline_queue_engine_subsystem)
        pprint.pprint(instance.unreal_service.get_properties_from_object(uobject=movie_pipeline_queue_engine_subsystem))

        # find rendering functions
        movie_pipeline_queue_engine_subsystem_static_class = instance.unreal_service.get_static_class(class_name="UMoviePipelineQueueEngineSubsystem")
        allocate_job_func = instance.unreal_service.find_function_by_name(uclass=movie_pipeline_queue_engine_subsystem_static_class, function_name="AllocateJob")
        render_job_func = instance.unreal_service.find_function_by_name(uclass=movie_pipeline_queue_engine_subsystem_static_class, function_name="RenderJob")
        is_rendering_func = instance.unreal_service.find_function_by_name(uclass=movie_pipeline_queue_engine_subsystem_static_class, function_name="IsRendering")

        movie_pipeline_executor_job_static_class = instance.unreal_service.get_static_class(class_name="UMoviePipelineExecutorJob")
        set_configuration_func = instance.unreal_service.find_function_by_name(uclass=movie_pipeline_executor_job_static_class, function_name="SetConfiguration")

        # load level sequence
        level_sequence = instance.unreal_service.load_object(class_name="ULevelSequence", outer=0, name="/Game/Spear/Scenes/apartment_0000/Cinematic/LS_DebugLevelSequence.LS_DebugLevelSequence")
        spear.log("level_sequence: ", level_sequence)
        pprint.pprint(instance.unreal_service.get_properties_from_object(uobject=level_sequence))

        # allocate job
        return_values = instance.unreal_service.call_function(uobject=movie_pipeline_queue_engine_subsystem, ufunction=allocate_job_func, args={"InSequence": spear.to_ptr(level_sequence)})
        movie_pipeline_executor_job = spear.to_handle(string=return_values["ReturnValue"])
        spear.log("movie_pipeline_executor_job: ", movie_pipeline_executor_job)
        pprint.pprint(instance.unreal_service.get_properties_from_object(uobject=movie_pipeline_executor_job))

        # load configuration
        movie_pipeline_primary_config = instance.unreal_service.load_object(class_name="UMoviePipelinePrimaryConfig", outer=0, name="/Game/Spear/Scenes/apartment_0000/Cinematic/MPPC_DebugMoviePipelinePrimaryConfig.MPPC_DebugMoviePipelinePrimaryConfig")
        spear.log("movie_pipeline_primary_config: ", movie_pipeline_primary_config)
        pprint.pprint(instance.unreal_service.get_properties_from_object(uobject=movie_pipeline_primary_config))

        #
        # On Windows, it is possible to access Unreal's path tracer if the UMoviePipelinePrimaryConfig object
        # used for rendering has been configured to enable path tracer output.
        #

        # ray_tracer_enable_cvar = instance.unreal_service.find_console_variable_by_name(console_variable_name="r.RayTracing.Enable")
        # ray_tracer_enable_cvar_value = instance.unreal_service.get_console_variable_value_as_int(cvar=ray_tracer_enable_cvar)
        # instance.unreal_service.set_console_variable_value(cvar=ray_tracer_enable_cvar, value=1)
        # movie_pipeline_primary_config = instance.unreal_service.load_object(class_name="UMoviePipelinePrimaryConfig", outer=0, name="/Game/Spear/Scenes/apartment_0000/Cinematic/MPPC_DebugMoviePipelinePrimaryConfigWithPathTracer.MPPC_DebugMoviePipelinePrimaryConfigWithPathTracer")
        # spear.log("movie_pipeline_primary_config: ", movie_pipeline_primary_config)
        # pprint.pprint(instance.unreal_service.get_properties_from_object(uobject=movie_pipeline_primary_config))

        # set job's configuration
        instance.unreal_service.call_function(uobject=movie_pipeline_executor_job, ufunction=set_configuration_func, args={"InPreset": spear.to_ptr(movie_pipeline_primary_config)})

        # render job
        instance.unreal_service.call_function(uobject=movie_pipeline_queue_engine_subsystem, ufunction=render_job_func, args={"InJob": spear.to_ptr(movie_pipeline_executor_job)})

    with instance.end_frame():
        pass

    # check rendering status
    is_rendering = True
    while is_rendering:
        time.sleep(1.0)
        with instance.begin_frame():
            return_values = instance.unreal_service.call_function(uobject=movie_pipeline_queue_engine_subsystem, ufunction=is_rendering_func)
            is_rendering = return_values["ReturnValue"]
            spear.log("is_rendering: ", is_rendering)
        with instance.end_frame():
            pass

    #
    # On Windows, if we turned on hardware ray tracing to access Unreal's path tracer, we should restore
    # hardware ray tracing to its previous state here.
    #

    # with instance.begin_frame():
    #     instance.unreal_service.set_console_variable_value(cvar=ray_tracer_enable_cvar, value=ray_tracer_enable_cvar_value)
    # with instance.end_frame():
    #     pass

    spear.log("Done.")
