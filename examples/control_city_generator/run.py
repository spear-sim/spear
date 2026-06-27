#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import time
import numpy as np
import pandas as pd
import scipy.interpolate
import scipy.spatial.transform
import spear


_PCG_VOLUME_NAMES_TO_GENERATE = [
    "City_Generator_PCG_2_Districts",
    "City_Generator_PCG_3_Lots",
    "City_Generator_PCG_4_Ground",
    "City_Generator_PCG_5_Buildings",
    "City_Generator_PCG_6_Left_over_lots",
    "City_Generator_PCG_7_city_Contour",
]

# Wall-clock duration of the camera flythrough, in seconds.
_FLYTHROUGH_DURATION_S = 20.0


if __name__ == "__main__":

    # Load knots from CSV as homogeneous row vectors [x, y, 1] in normalized letter coordinates.
    knots_file = os.path.join(os.path.dirname(__file__), "knots.csv")
    knots_df = pd.read_csv(knots_file, comment="#")
    knots_norm_x = knots_df["x"].to_numpy()
    knots_norm_y = knots_df["y"].to_numpy()
    knots_norm = np.column_stack([knots_norm_x, knots_norm_y, np.ones_like(knots_norm_x)])  # (N, 3)

    # City_Shape's center, ground Z, and horizontal extent. These are deterministic, so we
    # hardcode them rather than reading the spline back at runtime.
    city_center_x = -475.0
    city_center_y = 1358.0
    city_ground_z = 0.0
    city_width    = 104000.0

    # Scale letters to match the city shape's horizontal extent.
    scale = 2.75 * city_width / (knots_norm_x.max() - knots_norm_x.min())  # cm per letter unit

    # Center of the normalized letter coordinates: [x, y] midpoint of their bounding box.
    knots_center_norm = (knots_norm[:, :2].min(axis=0) + knots_norm[:, :2].max(axis=0)) / 2.0

    # Affine transform from normalized letter coordinates to 3D UE world coordinates: letter +Y
    # maps to UE +X and letter +X to UE +Y (so the word runs along UE +Y), scaled and centered
    # on the city and laid flat at ground level.
    T_world_from_norm = np.matrix([
        [0.0,   scale, city_center_x - scale*knots_center_norm[1]],
        [scale, 0.0,   city_center_y - scale*knots_center_norm[0]],
        [0.0,   0.0,   city_ground_z]])

    P_norm = np.matrix(knots_norm).T    # (3, N)
    P_world = T_world_from_norm*P_norm  # (3, N)
    knots_world = P_world.T.A           # (N, 3)

    # Animated camera flythrough over the generated SPEAR city. Keyframes are hardcoded world
    # poses in camera_keyframes.csv: a time, a position, and a camera-to-world rotation matrix
    # (camera_rotation_ij = row i, column j; columns are the camera's forward/right/up axes in
    # world space). They depend only on City_Shape, which is deterministic.
    keyframes_file = os.path.join(os.path.dirname(__file__), "camera_keyframes.csv")
    keyframes_df = pd.read_csv(keyframes_file, comment="#")

    keyframe_times     = keyframes_df["time"].to_numpy()
    keyframe_positions = keyframes_df[["position_x", "position_y", "position_z"]].to_numpy()
    keyframe_rotations = keyframes_df[[f"camera_rotation_{i}{j}" for i in range(3) for j in range(3)]].to_numpy().reshape(-1, 3, 3)

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance()
    game = instance.get_game()

    # Move arteries far out of the way (they can't be deleted but would interfere with city generation).
    with instance.begin_frame():
        for artery_name in ["Artery_1", "Artery_2"]:
            artery = game.unreal_service.find_actor_by_name(actor_name=artery_name, uclass="AActor")
            root = game.unreal_service.get_component_by_name(actor=artery, component_name="BrushComponent0.DefaultSceneRoot", uclass="USceneComponent")
            root.SetMobility(NewMobility="Movable")
            artery.K2_SetActorLocation(NewLocation={"X": 1000000.0, "Y": 1000000.0, "Z": 0.0}, bSweep=False, bTeleport=True)
    with instance.end_frame():
        pass
    spear.log("Moved arteries.")

    # Replace City_Shape spline with SPEAR letter knots.
    with instance.begin_frame():
        city_shape = game.unreal_service.find_actor_by_name(actor_name="City_Shape", uclass="AActor")
        spline = game.unreal_service.get_component_by_class(actor=city_shape, uclass="USplineComponent")
        spline.ClearSplinePoints(bUpdateSpline=False)
        for i, knot_world in enumerate(knots_world):
            is_last = (i == len(knots_world) - 1)
            spline.AddSplinePoint(Position=spear.math.to_spear_vector_from_numpy_array(numpy_array=knot_world), CoordinateSpace="World", bUpdateSpline=is_last)
    with instance.end_frame():
        pass
    spear.log(f"Set {len(knots_world)} spline points.")

    # Regenerate PCG_2 through PCG_7 in dependency order. Generate(bForce=True) ignores the PCG
    # cache, and the modified City_Shape spline changes each graph's input hash, so a separate
    # FlushCache is unnecessary here.
    with instance.begin_frame():
        for name in _PCG_VOLUME_NAMES_TO_GENERATE:
            pcg_actor = game.unreal_service.find_actor_by_name(actor_name=name, uclass="AActor")
            pcg_comp = pcg_actor.pCGComponent.get()
            pcg_comp.Generate(bForce=True)
        spear.log("PCG regeneration triggered.")
    with instance.end_frame():
        pass

    # Read the current viewport pose so the flight starts from where the camera is now, and
    # get the spectator pawn (its view is driven by the player controller's control rotation).
    with instance.begin_frame():
        viewport_desc = game.rendering_service.get_current_viewport_desc(only_get_pose=True)
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")
        player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)
        pawn = player_controller.K2_GetPawn()
    with instance.end_frame():
        pass

    p_start = spear.math.to_numpy_array_from_spear_vector(spear_vector=viewport_desc["camera_location"])
    R_start = spear.math.to_numpy_matrix_from_spear_rotator(spear_rotator=viewport_desc["camera_rotation"])

    # Prepend the live viewport pose as keyframe 0 (time 0) so the motion starts smoothly from
    # the current view; the remaining keyframes come from the CSV.
    kt = np.concatenate([[0.0], keyframe_times])
    kp = np.vstack([p_start, keyframe_positions])
    kr_matrices = np.vstack([R_start[np.newaxis], keyframe_rotations])

    # Smooth splines for position (cubic) and rotation (cubic rotation spline). The clamped
    # boundary condition forces zero velocity at the two endpoints only, giving ease-in at
    # the start and ease-out at the stop while keeping full speed through the middle.
    pos_spline = scipy.interpolate.CubicSpline(kt, kp, bc_type="clamped")
    rot_spline = scipy.spatial.transform.RotationSpline(kt, scipy.spatial.transform.Rotation.from_matrix(kr_matrices))

    # Animate: sample the splines by elapsed wall-clock time so the flythrough always takes
    # _FLYTHROUGH_DURATION_S regardless of render speed (faster machines just sample it finer).
    spear.log(f"Starting {_FLYTHROUGH_DURATION_S:.0f}s flythrough.")
    start_time = time.time()
    while True:
        frame_t = min((time.time() - start_time) / _FLYTHROUGH_DURATION_S, 1.0)
        pos = pos_spline(frame_t)
        rot_dict = spear.math.to_spear_rotator_from_numpy_matrix(numpy_matrix=rot_spline(frame_t).as_matrix())

        spear.log(f"Setting camera location and rotation: [{pos[0]:.0f}, {pos[1]:.0f}, {pos[2]:.0f}], "
                  f"(pitch={rot_dict['Pitch']:.1f}, yaw={rot_dict['Yaw']:.1f}, roll={rot_dict['Roll']:.1f})")
        with instance.begin_frame():
            pawn.K2_SetActorLocation(NewLocation=spear.math.to_spear_vector_from_numpy_array(numpy_array=pos), bSweep=False, bTeleport=True)
            player_controller.SetControlRotation(NewRotation=rot_dict)
        with instance.end_frame():
            pass

        if frame_t >= 1.0:
            break

    spear.log("Done.")
