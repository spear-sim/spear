# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

# Before running this example, you should have a good understanding of the example in render_image/run.py

import cv2
import math
import numpy as np
import os
import spear

num_frames = 600

if __name__ == "__main__":

    # define the number of cameras in the grid
    rows = 3
    cols = 3

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)

    images_directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), "images")
    if not os.path.exists(images_directory):
        os.makedirs(images_directory)

    camera_offsets = []
    for i in range(rows):
        for j in range(cols):
            # These offsets define the spatial displacement of the cameras in the grid
            # relative to the parent actor’s pose.
            offset = {"x": 0, "y": (j - cols / 2) * 40, "z": -(i - rows / 2) * 40}
            camera_offsets.append(offset)

    # initialize actors and components
    with instance.begin_frame():
        # find functions
        actor_static_class = instance.unreal_service.get_static_class(class_name="AActor")
        set_actor_location_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="K2_SetActorLocation")
        set_actor_rotation_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="K2_SetActorRotation")

        gameplay_statics_static_class = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
        get_player_controller_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, function_name="GetPlayerController")

        sp_scene_capture_component_2d_static_class = instance.unreal_service.get_static_class(class_name="USpSceneCaptureComponent2D")

        initialize_func = instance.unreal_service.find_function_by_name(
            uclass=sp_scene_capture_component_2d_static_class,
            function_name="Initialize",
        )
        finalize_func = instance.unreal_service.find_function_by_name(
            uclass=sp_scene_capture_component_2d_static_class,
            function_name="Terminate",
        )

        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_static_class, create_if_needed=False)

        bp_camera_sensor_uclass = instance.unreal_service.load_object(
            class_name="UClass",
            outer=0,
            name="/SpComponents/Blueprints/BP_Camera_Sensor.BP_Camera_Sensor_C",
        )

        # We will control the pose of the camera rig with a single parent actor
        parent_actor = instance.unreal_service.spawn_actor_from_class(uclass=actor_static_class)

        scene_component_static_class = instance.unreal_service.get_static_class(class_name="USceneComponent")

        # Create the parent actor’s root scene component.
        parent_root_component = instance.unreal_service.create_scene_component_by_class_from_actor(
            scene_component_class=scene_component_static_class,
            owner=parent_actor,
            scene_component_name="DefaultSceneRoot",
        )

        attach_to_component_func = instance.unreal_service.find_function_by_name(
            uclass=actor_static_class,
            function_name="K2_AttachToComponent",
        )

        bp_camera_sensor_actors = []
        final_tone_curve_hdr_components = []
        for offset in camera_offsets:
            bp_camera_sensor_actor = instance.unreal_service.spawn_actor_from_class(uclass=bp_camera_sensor_uclass)

            # Set the sensor actor’s location (offset relative to the eventual parent).
            instance.unreal_service.call_function(
                uobject=bp_camera_sensor_actor,
                ufunction=set_actor_location_func,
                args={"NewLocation": offset},
            )

            # Get the component that renders the camera view.
            final_tone_curve_hdr_component = instance.unreal_service.get_component_by_name(
                class_name="USceneComponent",
                actor=bp_camera_sensor_actor,
                component_name="DefaultSceneRoot.final_tone_curve_hdr",
            )

            # Instead of calling our own attach function, use the existing UFUNCTION.
            # Retrieve the sensor actor’s root scene component.
            child_root_component = instance.unreal_service.get_component_by_name(
                class_name="USceneComponent",
                actor=bp_camera_sensor_actor,
                component_name="DefaultSceneRoot",
            )
            # Specify attachment rules as UENUM arguments (passed as strings).
            attachment_args = {
                "Parent": spear.to_ptr(parent_root_component),
                "SocketName": "",
                "LocationRule": "KeepRelative",
                "RotationRule": "KeepRelative",
                "ScaleRule": "KeepRelative",
                "bWeldSimulatedBodies": False,
            }
            instance.unreal_service.call_function(
                uobject=bp_camera_sensor_actor,
                ufunction=attach_to_component_func,
                args=attachment_args,
            )

            bp_camera_sensor_actors.append(bp_camera_sensor_actor)
            final_tone_curve_hdr_components.append(final_tone_curve_hdr_component)

        # Configure the final_tone_curve_hdr component to match the viewport (width, height, FOV, post-processing settings, etc).

        post_process_volume = instance.unreal_service.find_actor_by_type(class_name="APostProcessVolume")
        return_values = instance.unreal_service.call_function(
            uobject=gameplay_statics_default_object,
            ufunction=get_player_controller_func,
            args={"PlayerIndex": 0},
        )
        player_controller = spear.to_handle(string=return_values["ReturnValue"])
        player_camera_manager_desc = instance.unreal_service.find_property_by_name_on_object(uobject=player_controller, property_name="PlayerCameraManager")
        player_camera_manager_string = instance.unreal_service.get_property_value(property_desc=player_camera_manager_desc)
        player_camera_manager = spear.to_handle(string=player_camera_manager_string)

        viewport_size = instance.engine_service.get_viewport_size()
        viewport_x = viewport_size[0]
        viewport_y = viewport_size[1]
        viewport_aspect_ratio = viewport_x / viewport_y
        view_target_pov_desc = instance.unreal_service.find_property_by_name_on_object(uobject=player_camera_manager, property_name="ViewTarget.POV")
        view_target_pov = instance.unreal_service.get_property_value(property_desc=view_target_pov_desc)

        fov = view_target_pov["fOV"] * math.pi / 180.0
        half_fov = fov / 2.0
        half_fov_adjusted = math.atan(math.tan(half_fov) * viewport_aspect_ratio / view_target_pov["aspectRatio"])
        fov_adjusted = half_fov_adjusted * 2.0
        fov_adjusted_degrees = fov_adjusted * 180.0 / math.pi

        volume_settings_desc = instance.unreal_service.find_property_by_name_on_object(uobject=post_process_volume, property_name="Settings")
        volume_settings = instance.unreal_service.get_property_value(property_desc=volume_settings_desc)

        # The pose of the multi-camera rig is controlled by the parent actor
        instance.unreal_service.call_function(
            uobject=parent_actor,
            ufunction=set_actor_location_func,
            args={"NewLocation": view_target_pov["location"]},
        )
        instance.unreal_service.call_function(
            uobject=parent_actor,
            ufunction=set_actor_rotation_func,
            args={"NewRotation": view_target_pov["rotation"]},
        )

        # For the sake of demonstration, we'll show moving the parent actor
        # will move the entire grid of cameras
        actor_rotation_matrix = spear.to_matrix_from_rotator(view_target_pov["rotation"])
        actor_forward_vector = actor_rotation_matrix[:, 0]
        actor_right_vector = actor_rotation_matrix[:, 1]
        actor_up_vector = actor_rotation_matrix[:, 2]

        final_tone_curve_hdr_component_shared_memory_handles_list = []
        for (
            bp_camera_sensor_actor,
            final_tone_curve_hdr_component,
        ) in zip(bp_camera_sensor_actors, final_tone_curve_hdr_components):
            width_desc = instance.unreal_service.find_property_by_name_on_object(uobject=final_tone_curve_hdr_component, property_name="Width")
            height_desc = instance.unreal_service.find_property_by_name_on_object(uobject=final_tone_curve_hdr_component, property_name="Height")
            fov_angle_desc = instance.unreal_service.find_property_by_name_on_object(uobject=final_tone_curve_hdr_component, property_name="FOVAngle")
            component_settings_desc = instance.unreal_service.find_property_by_name_on_object(
                uobject=final_tone_curve_hdr_component,
                property_name="PostProcessSettings",
            )
            instance.unreal_service.set_property_value(property_desc=width_desc, property_value=viewport_x)
            instance.unreal_service.set_property_value(property_desc=height_desc, property_value=viewport_y)
            instance.unreal_service.set_property_value(property_desc=fov_angle_desc, property_value=fov_adjusted_degrees)
            instance.unreal_service.set_property_value(property_desc=component_settings_desc, property_value=volume_settings)

            instance.unreal_service.call_function(
                uobject=final_tone_curve_hdr_component,
                ufunction=initialize_func,
            )
            final_tone_curve_hdr_component_shared_memory_handles = instance.sp_func_service.create_shared_memory_handles_for_object(
                uobject=final_tone_curve_hdr_component
            )
            final_tone_curve_hdr_component_shared_memory_handles_list.append(final_tone_curve_hdr_component_shared_memory_handles)

    with instance.end_frame():
        pass

    for frame_idx in range(num_frames):
        with instance.begin_frame():
            # move the parent actor around
            location = view_target_pov["location"]
            t = frame_idx / 30 * 2 * np.pi

            # walk foward along the lookat direction, but don't change the height above the floor
            x = frame_idx
            # Add a figure 8 pattern in the camera plane
            y = np.sin(t)
            z = np.sin(t) * np.cos(t)

            new_location = {
                "x": float(location["x"] + actor_forward_vector[0] * x + actor_right_vector[0] * 20 * y + actor_up_vector[0] * 20 * z),
                "y": float(location["y"] + actor_forward_vector[1] * x + actor_right_vector[1] * 20 * y + actor_up_vector[1] * 20 * z),
                "z": float(
                    location["z"]
                    # + actor_forward_vector[2] * x # don't walk down into the floor
                    + actor_right_vector[2] * 20 * y
                    + actor_up_vector[2] * 20 * z
                ),
            }

            # move the parent actor
            instance.unreal_service.call_function(
                uobject=parent_actor,
                ufunction=set_actor_location_func,
                args={"NewLocation": new_location},
            )

        with instance.end_frame():
            # build a montage of the rendered frames
            viewport_y = int(viewport_y)
            viewport_x = int(viewport_x)
            canvas = np.zeros((viewport_y * rows, viewport_x * cols, 4), dtype=np.uint8)
            for idx, (
                final_tone_curve_hdr_component_shared_memory_handles,
                final_tone_curve_hdr_component,
            ) in enumerate(
                zip(
                    final_tone_curve_hdr_component_shared_memory_handles_list,
                    final_tone_curve_hdr_components,
                )
            ):
                i = idx // cols
                j = idx % cols
                return_values = instance.sp_func_service.call_function(
                    uobject=final_tone_curve_hdr_component,
                    function_name="read_pixels",
                    uobject_shared_memory_handles=final_tone_curve_hdr_component_shared_memory_handles,
                )
                data = return_values["arrays"]["data"]
                canvas[
                    i * viewport_y : (i + 1) * viewport_y,
                    j * viewport_x : (j + 1) * viewport_x,
                ] = data

        filename = os.path.join(images_directory, "%04d.png" % frame_idx)
        cv2.imwrite(filename, canvas)
        spear.log(filename)

    with instance.begin_frame():
        pass
    with instance.end_frame():
        for (
            bp_camera_sensor_actor,
            final_tone_curve_hdr_component,
            final_tone_curve_hdr_component_shared_memory_handles,
        ) in zip(
            bp_camera_sensor_actors,
            final_tone_curve_hdr_components,
            final_tone_curve_hdr_component_shared_memory_handles_list,
        ):
            instance.sp_func_service.destroy_shared_memory_handles_for_object(shared_memory_handles=final_tone_curve_hdr_component_shared_memory_handles)
            instance.unreal_service.call_function(
                uobject=final_tone_curve_hdr_component,
                ufunction=finalize_func,
            )
            instance.unreal_service.destroy_actor(actor=bp_camera_sensor_actor)

    spear.log("Done.")
