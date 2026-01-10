#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
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
    game = instance.get_game()

    with instance.begin_frame():

        # get UMoviePipelineQueueEngineSubsystem
        movie_pipeline_queue_engine_subsystem = game.unreal_service.get_engine_subsystem(uclass="UMoviePipelineQueueEngineSubsystem")
        spear.log("movie_pipeline_queue_engine_subsystem: ", movie_pipeline_queue_engine_subsystem)
        pprint.pprint(movie_pipeline_queue_engine_subsystem.get_properties())

        # load level sequence
        level_sequence = game.unreal_service.load_object(uclass="ULevelSequence", name="/Game/SPEAR/Scenes/apartment_0000/Cinematic/LS_DebugLevelSequence.LS_DebugLevelSequence")
        spear.log("level_sequence: ", level_sequence)
        pprint.pprint(level_sequence.get_properties())

        # load configuration
        movie_pipeline_primary_config = game.unreal_service.load_object(uclass="UMoviePipelinePrimaryConfig", name="/SpContent/Cinematic/MPPC_DefaultConfigWithLighting.MPPC_DefaultConfigWithLighting")
        spear.log("movie_pipeline_primary_config: ", movie_pipeline_primary_config)
        pprint.pprint(movie_pipeline_primary_config.get_properties())

        # allocate job
        movie_pipeline_executor_job = movie_pipeline_queue_engine_subsystem.AllocateJob(InSequence=level_sequence)
        spear.log("movie_pipeline_executor_job: ", movie_pipeline_executor_job)
        pprint.pprint(movie_pipeline_executor_job.get_properties())

        #
        # On Windows, it is possible to access Unreal's path tracer if the UMoviePipelinePrimaryConfig object
        # used for rendering has been configured to enable path tracer output.
        #

        # ray_tracing_enable_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.RayTracing.Enable")
        # ray_tracing_enable_cvar_value = game.unreal_service.get_console_variable_value_as_int(cvar=ray_tracing_enable_cvar)
        # game.unreal_service.set_console_variable_value(cvar=ray_tracing_enable_cvar, value=1)
        # movie_pipeline_primary_config = game.unreal_service.load_object(uclass="UMoviePipelinePrimaryConfig", name="/SpContent/Cinematic/MPPC_DefaultConfigWithPathTracer.MPPC_DefaultConfigWithPathTracer")
        # spear.log("movie_pipeline_primary_config: ", movie_pipeline_primary_config)
        # pprint.pprint(game.unreal_service.get_properties_from_object(uobject=movie_pipeline_primary_config))

        # set job's configuration
        movie_pipeline_executor_job.SetConfiguration(InPreset=movie_pipeline_primary_config)

        # render job
        movie_pipeline_queue_engine_subsystem.RenderJob(InJob=movie_pipeline_executor_job)

    with instance.end_frame():
        pass

    # check rendering status
    is_rendering = True
    while is_rendering:
        time.sleep(1.0)
        with instance.begin_frame():
            is_rendering = movie_pipeline_queue_engine_subsystem.IsRendering()
            spear.log("is_rendering: ", is_rendering)
        with instance.end_frame():
            pass

    #
    # On Windows, if we turned on hardware ray tracing to access Unreal's path tracer, we should restore
    # hardware ray tracing to its previous state here.
    #

    # with instance.begin_frame():
    #     game.unreal_service.set_console_variable_value(cvar=ray_tracing_enable_cvar, value=ray_tracing_enable_cvar_value)
    # with instance.end_frame():
    #     pass

    spear.log("Done.")
