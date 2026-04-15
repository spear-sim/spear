#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import colorsys
import matplotlib.pyplot as plt
import numpy as np
import os
import shutil
import spear


np.random.seed(0)

component_descs = \
[
    {
        "name": "final_tone_curve_hdr",
        "long_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "visualize_func": lambda data : data[:,:,[2,1,0]] # BGRA to RGB
    },
    {
        "name": "sp_object_ids_uint8",
        "long_name": "DefaultSceneRoot.sp_object_ids_uint8_",
        "visualize_func": lambda data : data[:,:,[2,1,0]] # BGRA to RGB
    },
    {
        "name": "sp_unlit_float16",
        "long_name": "DefaultSceneRoot.sp_unlit_float16_",
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]]/200.0, 0.0, 1.0)
    }
]


if __name__ == "__main__":

    # create output dir
    images_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images"))
    if os.path.exists(images_dir):
        spear.log("Directory exists, removing: ", images_dir)
        shutil.rmtree(images_dir, ignore_errors=True)
    os.makedirs(images_dir, exist_ok=True)

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    with instance.begin_frame():

        game.segmentation_service.initialize()

        sp_actor_component = game.get_unreal_object(uclass="USpActorComponent")
        sp_scene_component = game.get_unreal_object(uclass="USpSceneComponent")
        kismet_system_library = game.get_unreal_object(uclass="UKismetSystemLibrary")

        bounding_box_mesh = game.unreal_service.load_object(uclass="UStaticMesh", name="/Engine/BasicShapes/Cube.Cube")
        bounding_box_material = game.unreal_service.load_object(uclass="UMaterialInterface", name="/SpContent/Materials/M_LocalPositionMultipliedByObjectScale.M_LocalPositionMultipliedByObjectScale")

        # compute bounding boxes from scene actors, excluding proxy component managers
        actors = [ actor for actor in game.unreal_service.find_actors() if not actor.is_a(uclass="ASpProxyComponentManager") ]

        bounding_box_descs = [None]
        for actor in actors:
            static_mesh_components = game.unreal_service.get_components_by_class(actor=actor, uclass="UStaticMeshComponent")

            for static_mesh_component in static_mesh_components:
                local_bounds = static_mesh_component.GetLocalBounds(Min={"X": 0.0, "Y": 0.0, "Z": 0.0}, Max={"X": 0.0, "Y": 0.0, "Z": 0.0}, as_dict=True)
                local_min = spear.to_numpy_array_from_vector(vector=local_bounds["Min"], as_matrix=True)
                local_max = spear.to_numpy_array_from_vector(vector=local_bounds["Max"], as_matrix=True)
                local_center = (local_min + local_max)/2.0
                local_extent = (local_max - local_min)/2.0

                component_location = spear.to_numpy_array_from_vector(vector=static_mesh_component.K2_GetComponentLocation(), as_matrix=True)
                component_rotation = spear.to_numpy_matrix_from_rotator(rotator=static_mesh_component.K2_GetComponentRotation(), as_matrix=True)
                component_scale = np.matrix(np.diag(spear.to_numpy_array_from_vector(vector=static_mesh_component.K2_GetComponentScale())))

                world_center = component_location + component_rotation*component_scale*local_center
                world_extent = np.abs(component_scale*local_extent)

                bounding_box_descs.append({
                    "world_center": world_center,
                    "world_extent": world_extent,
                    "rotation": component_rotation})

        num_bounding_boxes = len(bounding_box_descs)

        # generate colors for bounding boxes
        bounding_box_colors = np.zeros((num_bounding_boxes, 3))
        for i in range(1, num_bounding_boxes):
            bounding_box_colors[i] = np.array(colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0))

        # debug draw bounding boxes (skip index 0 which is the "none" entry)
        for desc, color in zip(bounding_box_descs[1:], bounding_box_colors[1:]):
            kismet_system_library.DrawDebugBox(
                Center=spear.to_vector_from_numpy_array(array=desc["world_center"]),
                Extent=spear.to_vector_from_numpy_array(array=desc["world_extent"]),
                LineColor={"R": float(color[0]), "G": float(color[1]), "B": float(color[2]), "A": 1.0},
                Rotation=spear.to_rotator_from_numpy_matrix(matrix=desc["rotation"]),
                Duration=60.0,
                Thickness=2.0)

        # spawn bounding box mesh actors (visible in scene capture only, skip index 0)
        for desc in bounding_box_descs[1:]:
            bounding_box_actor = game.unreal_service.spawn_actor(
                uclass="AStaticMeshActor",
                location=spear.to_vector_from_numpy_array(array=desc["world_center"]),
                rotation=spear.to_rotator_from_numpy_matrix(matrix=desc["rotation"]),
                spawn_parameters={"SpawnCollisionHandlingOverride": "AlwaysSpawn"})

            static_mesh_component = game.unreal_service.get_component_by_class(actor=bounding_box_actor, uclass="UStaticMeshComponent")

            static_mesh_component.SetMobility(NewMobility="Movable")
            static_mesh_component.SetStaticMesh(NewMesh=bounding_box_mesh)
            static_mesh_component.SetMaterial(ElementIndex=0, Material=bounding_box_material)

            static_mesh_component.SetVisibleInSceneCaptureOnly(bValue=True)
            static_mesh_component.bCastDynamicShadow = False
            static_mesh_component.bCastStaticShadow = False
            static_mesh_component.bAffectDistanceFieldLighting = False
            static_mesh_component.bAffectDynamicIndirectLighting = False

            sp_actor_component.SetCanEverAffectNavigation(ActorComponent=static_mesh_component, bCanEverAffectNavigation=False)
            static_mesh_component.SetCollisionEnabled(NewType="NoCollision")
            static_mesh_component.SetCollisionProfileName(InCollisionProfileName="NoCollision") # needed for editor worlds
            static_mesh_component.SetGenerateOverlapEvents(bInGenerateOverlapEvents=False)

            bounding_box_actor.SetActorScale3D(NewScale3D=spear.to_vector_from_numpy_array(array=desc["world_extent"]/50.0))

            sp_actor_component.MarkRenderStateDirty(ActorComponent=static_mesh_component)
            sp_scene_component.MarkRenderTransformDirty(SceneComponent=static_mesh_component)

            desc["bounding_box_actor"] = bounding_box_actor

        # spawn camera sensor and get components
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)

        components = []
        for component_desc in component_descs:
            component_desc["component"] = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name=component_desc["long_name"], uclass="USpSceneCaptureComponent2D")
            components.append(component_desc["component"])

        # configure camera to match viewport
        viewport_info = game.rendering_service.get_current_viewport_info()
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=components, viewport_info=viewport_info)

        # configure capture component visibility routing

        component_desc_map = { desc["name"]: desc for desc in component_descs }
        bounding_box_actors = [ None if desc is None else desc["bounding_box_actor"] for desc in bounding_box_descs ]

        # hide bounding boxes in the beauty pass
        final_tone_curve_hdr_component = component_desc_map["final_tone_curve_hdr"]["component"]
        final_tone_curve_hdr_component.PrimitiveRenderMode = "PRM_RenderScenePrimitives" # use HiddenActors
        final_tone_curve_hdr_component.HiddenActors = bounding_box_actors[1:]

        # configure the unlit pass to only render bounding box actors
        unlit_component = component_desc_map["sp_unlit_float16"]["component"]
        unlit_component.PrimitiveRenderMode = "PRM_UseShowOnlyList" # use ShowOnlyActors
        unlit_component.ShowOnlyActors = bounding_box_actors[1:]

        # configure the segmentation pass to only render bounding box actors
        game.segmentation_service.set_allowed_actors(allowed_actors=bounding_box_actors[1:])

        # initialize components
        for component_desc in component_descs:
            component_desc["component"].Initialize()
            component_desc["component"].initialize_sp_funcs()

    with instance.end_frame():
        pass

    # needed in case our custom material is still loading
    game.async_loading_service.wait_for_engine_idle()

    # let temporal anti-aliasing etc accumulate additional information across multiple frames
    instance.step(num_frames=2)

    # get rendered frame
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component_desc in component_descs:
            data_bundle = component_desc["component"].read_pixels()
            component_desc["data"] = data_bundle["arrays"]["data"]

        object_ids_bgra_uint8_image = component_desc_map["sp_object_ids_uint8"]["data"]
        proxy_id_image, proxy_id_descs = game.segmentation_service.get_segmentation_data(object_ids_bgra_uint8_image=object_ids_bgra_uint8_image)

    # save images
    for component_desc in component_descs:
        data = component_desc["data"]
        image_file = os.path.realpath(os.path.join(images_dir, f"{component_desc['name']}.png"))
        image = component_desc["visualize_func"](data=data)
        spear.log("Saving image: ", image_file)
        plt.imsave(image_file, image)

    # save segmentation image with colors matching debug draw
    bounding_box_handles = [ 0 if actor is None else actor.uobject for actor in bounding_box_actors ]
    assert all([ proxy_id_desc["actor"] in bounding_box_handles for proxy_id_desc in proxy_id_descs ])
    proxy_bounding_box_ids = np.array([ bounding_box_handles.index(proxy_id_desc["actor"]) for proxy_id_desc in proxy_id_descs ])
    bounding_box_colors_uint8 = np.round(bounding_box_colors*255.0).astype(np.uint8)
    segmentation_image = bounding_box_colors_uint8[proxy_bounding_box_ids[proxy_id_image]]
    image_file = os.path.realpath(os.path.join(images_dir, "segmentation.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, segmentation_image)

    # cleanup
    with instance.begin_frame():
        pass

    with instance.end_frame():
        for component_desc in component_descs:
            component_desc["component"].terminate_sp_funcs()
            component_desc["component"].Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

        for bounding_box_actor in bounding_box_actors[1:]:
            game.unreal_service.destroy_actor(actor=bounding_box_actor)

        game.segmentation_service.terminate()

    spear.log("Done.")
