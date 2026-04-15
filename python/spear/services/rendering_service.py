#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import math
import spear


class RenderingService(spear.Service):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, config):
        assert sp_func_service.is_top_level_service()
        assert unreal_service.is_top_level_service()

        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            config=config)

    def align_camera_with_viewport(self, camera_sensor, camera_components, viewport_info, widths=None, heights=None, only_align_pose=False, post_processing_components=None, post_processing_volumes=None):

        camera_sensor.K2_SetActorLocationAndRotation(NewLocation=viewport_info["camera_location"], NewRotation=viewport_info["camera_rotation"])

        if only_align_pose:
            return

        # normalize camera_components to a list
        if isinstance(camera_components, dict):
            camera_components = list(camera_components.values())
        elif not isinstance(camera_components, list):
            camera_components = [camera_components]

        # normalize widths and heights to lists
        viewport_size_x = viewport_info["viewport_size_x"]
        viewport_size_y = viewport_info["viewport_size_y"]

        if widths is None:
            widths = [viewport_size_x]*len(camera_components)
        elif not isinstance(widths, list):
            widths = [widths]*len(camera_components)

        if heights is None:
            heights = [viewport_size_y]*len(camera_components)
        elif not isinstance(heights, list):
            heights = [heights]*len(camera_components)

        # normalize post_processing_components and post_processing_volumes to lists
        if post_processing_components is not None:
            if not isinstance(post_processing_components, list):
                post_processing_components = [post_processing_components]

            if post_processing_volumes is None:
                post_processing_volumes = viewport_info["post_process_volumes"]
                if len(post_processing_volumes) == 1:
                    post_processing_volumes = post_processing_volumes*len(post_processing_components)
            elif not isinstance(post_processing_volumes, list):
                post_processing_volumes = [post_processing_volumes]*len(post_processing_components)

        # configure camera components
        for camera_component, w, h in zip(camera_components, widths, heights):
            camera_component.Width = w
            camera_component.Height = h

            if viewport_info["is_perspective"]:
                render_target_aspect_ratio = w/h
                fov = viewport_info["fov_degrees"]*math.pi/180.0
                half_fov = fov/2.0
                half_fov_adjusted = math.atan(math.tan(half_fov)*render_target_aspect_ratio/viewport_info["aspect_ratio"])
                fov_adjusted_degrees = half_fov_adjusted*2.0*180.0/math.pi
                camera_component.ProjectionType = "Perspective"
                camera_component.FOVAngle = fov_adjusted_degrees
            else:
                camera_component.ProjectionType = "Orthographic"
                camera_component.OrthoWidth = viewport_info["ortho_width"]

        # apply post-processing settings
        if post_processing_components is not None:
            for camera_component, volume in zip(post_processing_components, post_processing_volumes):
                camera_component.PostProcessSettings = volume.Settings.get()


class GameRenderingService(RenderingService):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, engine_globals_service, config):
        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            config=config)

        self._engine_globals_service = engine_globals_service
        self._game_viewport_client = None
        self._gameplay_statics = None
        self._sp_game_viewport = None

    def initialize(self):
        engine = self._engine_globals_service.get_engine()
        self._game_viewport_client = engine.GameViewport.get()
        self._gameplay_statics = self.get_unreal_object(uclass="UGameplayStatics")
        self._sp_game_viewport = self.get_unreal_object(uclass="USpGameViewportClient")

    def get_current_viewport_info(self, only_get_pose=False):
        player_controller = self._gameplay_statics.GetPlayerController(PlayerIndex=0)
        pov = player_controller.PlayerCameraManager.ViewTarget.POV.get()

        if only_get_pose:
            return {
                "camera_location": pov["location"],
                "camera_rotation": pov["rotation"]}

        else:
            viewport_size = self._sp_game_viewport.GetViewportSize(GameViewportClient=self._game_viewport_client, as_dict=True)["ViewportSize"]
            is_perspective = pov["projectionMode"] == "Perspective"

            if is_perspective:
                viewport_aspect_ratio = viewport_size["x"]/viewport_size["y"]
                half_fov = pov["fOV"]*math.pi/360.0
                fov_x_degrees = math.atan(math.tan(half_fov)*viewport_aspect_ratio/pov["aspectRatio"])*360.0/math.pi
                fov_y_degrees = math.atan(math.tan(fov_x_degrees*math.pi/360.0)/viewport_aspect_ratio)*360.0/math.pi

            return {
                "viewport_size_x": viewport_size["x"],
                "viewport_size_y": viewport_size["y"],
                "camera_location": pov["location"],
                "camera_rotation": pov["rotation"],
                "is_perspective": is_perspective,
                "fov_x_degrees": fov_x_degrees if is_perspective else None,
                "fov_y_degrees": fov_y_degrees if is_perspective else None,
                "fov_degrees": pov["fOV"] if is_perspective else None,
                "aspect_ratio": pov["aspectRatio"] if is_perspective else None,
                "ortho_width": pov["orthoWidth"] if not is_perspective else None,
                "post_process_volumes": self.unreal_service.find_actors_by_class(uclass="APostProcessVolume")}


class EditorRenderingService(RenderingService):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, config):
        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            config=config)

        self._sp_editor_engine = None

    def initialize(self):
        self._sp_editor_engine = self.get_unreal_object(uclass="USpEditorEngine")

    def get_current_viewport_info(self, only_get_pose=False):
        viewport_client_descs = self._sp_editor_engine.GetLevelViewportClients()
        editing_descs = [ desc for desc in viewport_client_descs if desc["bIsCurrentLevelEditing"] ]
        assert len(editing_descs) == 1
        client = editing_descs[0]

        if only_get_pose:
            return {
                "camera_location": client["cameraLocation"],
                "camera_rotation": client["cameraRotation"]}

        else:
            is_perspective = client["bIsPerspective"]

            if is_perspective:
                viewport_aspect_ratio = client["viewportSize"]["x"]/client["viewportSize"]["y"]
                half_fov = client["viewFOV"]*math.pi/360.0
                fov_x_degrees = math.atan(math.tan(half_fov)*viewport_aspect_ratio/client["aspectRatio"])*360.0/math.pi
                fov_y_degrees = math.atan(math.tan(fov_x_degrees*math.pi/360.0)/viewport_aspect_ratio)*360.0/math.pi

            return {
                "viewport_size_x": client["viewportSize"]["x"],
                "viewport_size_y": client["viewportSize"]["y"],
                "camera_location": client["cameraLocation"],
                "camera_rotation": client["cameraRotation"],
                "is_perspective": is_perspective,
                "fov_x_degrees": fov_x_degrees if is_perspective else None,
                "fov_y_degrees": fov_y_degrees if is_perspective else None,
                "fov_degrees": client["viewFOV"] if is_perspective else None,
                "aspect_ratio": client["aspectRatio"] if is_perspective else None,
                "ortho_width": client["orthoZoom"] if not is_perspective else None,
                "post_process_volumes": self.unreal_service.find_actors_by_class(uclass="APostProcessVolume")}
