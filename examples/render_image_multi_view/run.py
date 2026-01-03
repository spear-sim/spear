# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

# Before running this example, you should have a good understanding of the example in render_image/run.py

import cv2
import math
import numpy as np
import os
import spear


num_frames = 300
num_rows = 3
num_cols = 3
camera_location = {"X": -125.0, "Y": 130.0, "Z": 245.0}
camera_rotator = {"Pitch": -20.0, "Yaw": 20.0, "Roll": 0.0}

rotator = {"Pitch": 0.0, "Yaw": 20.0, "Roll": 0.0}
p_world_init = spear.to_numpy_array_from_vector(vector=camera_location, as_matrix=True)
R_world_from_actor = spear.to_numpy_matrix_from_rotator(rotator=rotator, as_matrix=True)


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    images_directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), "images")
    if not os.path.exists(images_directory):
        os.makedirs(images_directory)

    # initialize actors and components
    with instance.begin_frame():

        # spawn camera sensor and get its components
        bp_multi_view_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_MultiViewCameraSensor.BP_MultiViewCameraSensor_C")
        bp_multi_view_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_multi_view_camera_sensor_uclass)
        components = game.unreal_service.get_components_by_class_as_dict(actor=bp_multi_view_camera_sensor, uclass="USpSceneCaptureComponent2D")
        assert len(components) == num_rows*num_cols

        bp_multi_view_camera_sensor.K2_SetActorLocation(NewLocation=camera_location)
        bp_multi_view_camera_sensor.K2_SetActorRotation(NewRotation=camera_rotator)

        post_process_volume = game.unreal_service.find_actor_by_class(uclass="APostProcessVolume")
        post_process_volume_settings = post_process_volume.Settings.get()

        # configure components
        component_names = sorted(components.keys())
        for c, component_name in enumerate(component_names):
            component = components[component_name]
            if c == 0:
                component_width = component.Width.get()
                component_height = component.Height.get()
            else:
                assert component.Width.get() == component_width
                assert component.Height.get() == component_height
            component.PostProcessSettings = post_process_volume_settings
            component.Initialize()
            component.initialize_sp_funcs()

    with instance.end_frame():
        pass

    for f in range(num_frames):
        with instance.begin_frame():

            t = f/30.0*2.0*np.pi
            x = float(f)
            y = 20.0*np.sin(t)
            z = 20.0*np.sin(t)*np.cos(t)

            p_actor = np.matrix([x, y, z]).T
            p_world = p_world_init + R_world_from_actor*p_actor
            bp_multi_view_camera_sensor.K2_SetActorLocation(NewLocation=spear.to_vector_from_numpy_array(array=p_world))

        with instance.end_frame():
            canvas = np.zeros((component_height*num_rows, component_width*num_cols, 4), dtype=np.uint8)
            for c, component_name in enumerate(component_names):
                component = components[component_name]
                i = int(c % num_cols)
                j = int(c / num_cols)
                return_values = component.read_pixels()
                data = return_values["arrays"]["data"]
                canvas[component_height*j : component_height*(j+1), component_width*i : component_width*(i+1)] = data

        filename = os.path.join(images_directory, f"{f:04}.png")
        spear.log("Saving image: ", filename)
        cv2.imwrite(filename, canvas)

    with instance.begin_frame():
        pass
    with instance.end_frame():
        for c, component_name in enumerate(component_names):
            component = components[component_name]
            component.terminate_sp_funcs()
            component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_multi_view_camera_sensor)

    spear.log("Done.")
