#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import ast
import cv2
import inspect
import math
import mcp
import numpy as np
import os
import spear
import sys
import traceback


#
# Constants
#

_BANNED_ATTRS = {"__bases__", "__builtins__", "__class__", "__code__", "__dict__", "__func__", "__globals__", "__locals__", "__subclasses__", "__wrapped__"}
_BANNED_CALLS = {"__import__", "compile", "delattr", "eval", "exec", "execute_file", "execute_file_across_frames", "getattr", "open", "save", "savez", "savez_compressed", "savetxt", "setattr", "tofile"}
_BANNED_NAMES = {"__builtins__", "__import__", "builtins", "importlib", "os", "pathlib", "shutil", "socket", "subprocess", "sys"}

_SAVE_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), "tmp", "spear-mcp"))
_MAX_IMAGE_DIM = 256

_STABLE_NAME_CAMERA_SENSOR = "__SP_CAMERA_SENSOR__"
_STABLE_NAME_PROXY_COMPONENT_MANAGER = "__SP_PROXY_COMPONENT_MANAGER__"
_STABLE_NAME_STABLE_NAME_MANAGER = "__SP_STABLE_NAME_MANAGER__"
_EDITOR_FOLDER_PATH = "__SPEAR_MCP__"


#
# Logging
#

_log_lines = []
_log_file = os.path.realpath(os.path.join(_SAVE_DIR, "mcp_server.log"))
_log_file_prev = os.path.realpath(os.path.join(_SAVE_DIR, "mcp_server.prev.log"))

def _rotate_log_file():
    os.makedirs(os.path.dirname(_log_file), exist_ok=True)
    if os.path.exists(_log_file_prev):
        os.remove(_log_file_prev)
    if os.path.exists(_log_file):
        os.rename(_log_file, _log_file_prev)

def _log(*args):
    frame = inspect.currentframe()
    outer = inspect.getouterframes(frame)
    filename = os.path.basename(outer[1].filename)
    lineno = f"{outer[1].lineno:04}"
    message = f"[SPEAR-MCP | {filename}:{lineno}] " + "".join([str(arg) for arg in args])
    _log_lines.append(message)
    with open(_log_file, "a") as f:
        f.write(message + "\n")

def _get_log():
    result = "\n".join(_log_lines)
    _log_lines.clear()
    return result

_rotate_log_file()


#
# Global state
#

_mcp = mcp.server.fastmcp.FastMCP("spear")

_instance = None
_game = None
_editor = None
_world_descs = None
_exec_namespace = {}


#
# MCP tools
#

@_mcp.tool()
def initialize():
    try:
        _initialize_instance()
        _log_instance_status()
        return _get_log()

    except Exception:
        _log(f"INTERNAL EXCEPTION:\n{traceback.format_exc()}")
        return _get_log()


@_mcp.tool()
def terminate():
    try:
        _terminate_instance()
        _log_instance_status()
        return _get_log()

    except Exception:
        _log(f"INTERNAL EXCEPTION:\n{traceback.format_exc()}")
        return _get_log()


@_mcp.tool()
def execute_code(code, save_images=None):
    try:
        success = _validate_code(code=code)
        if not success:
            _log_instance_status()
            return _get_log()

        success = _initialize_instance()
        if not success:
            _log_instance_status()
            return _get_log()

        success, world_desc = _initialize_actors()
        if not success:
            _log_instance_status()
            return _get_log()

        # execute code
        _log(f"Executing code:\n\n\n{code}\n\n")
        spear.register_log_func(func=_log)
        try:
            exec(code, _exec_namespace)
        except Exception:
            _log(f"EXCEPTION:\n{traceback.format_exc()}")
        finally:
            spear.unregister_log_func(func=_log)

        # attempt to clean up bad frame state
        if _instance._engine_service._frame_state == "idle":
            pass
        elif _instance._engine_service._frame_state == "executing_frame":
            with _instance.end_frame():
                pass
            _log("ERROR: Agent code executed a begin_frame block without a matching end_frame block, executing empty end_frame block...")
        else:
            _log(f"ERROR: Unexpected frame state after executing agent code: {_instance._engine_service._frame_state}")

        # we need to save images before terminating actors
        if save_images is not None:
            success = _save_images(variable_names=save_images)

        _terminate_actors(world_desc=world_desc)

        _log_instance_status()
        return _get_log()

    except Exception:
        _log(f"INTERNAL EXCEPTION:\n{traceback.format_exc()}")
        return _get_log()


@_mcp.tool()
def execute_editor_code(code):
    try:
        success = _validate_code(code=code)
        if not success:
            _log_instance_status()
            return _get_log()

        success = _initialize_instance()
        if not success:
            _log_instance_status()
            return _get_log()

        if _editor is None:
            _log("ERROR: Editor not available.")
            _log_instance_status()
            return _get_log()

        success, world_desc = _initialize_actors()
        if not success:
            _log_instance_status()
            return _get_log()

        # execute code
        _log(f"Executing editor code:\n\n\n{code}\n\n")
        spear.register_log_func(func=_log)
        try:
            with _instance.begin_frame():
                try:
                    _editor.python_service.execute_string(string=code, execution_scope="Public")
                except RuntimeError:
                    _log(f"EXCEPTION:\n{traceback.format_exc()}")
            with _instance.end_frame():
                pass
        finally:
            spear.unregister_log_func(func=_log)

        _terminate_actors(world_desc=world_desc)
        _log_instance_status()
        return _get_log()

    except Exception:
        _log(f"INTERNAL EXCEPTION:\n{traceback.format_exc()}")
        return _get_log()


@_mcp.tool()
def execute_editor_code_across_frames(code):
    try:
        success = _validate_code(code=code)
        if not success:
            _log_instance_status()
            return _get_log()

        success = _initialize_instance()
        if not success:
            _log_instance_status()
            return _get_log()

        if _editor is None:
            _log("ERROR: Editor not available.")
            _log_instance_status()
            return _get_log()

        success, world_desc = _initialize_actors()
        if not success:
            _log_instance_status()
            return _get_log()

        # execute code
        _log(f"Executing editor code across frames:\n\n\n{code}\n\n")
        spear.register_log_func(func=_log)
        try:
            try:
                _editor.python_service.execute_string_across_frames(string=code, execution_scope="Public")
            except RuntimeError:
                _log(f"EXCEPTION:\n{traceback.format_exc()}")
        finally:
            spear.unregister_log_func(func=_log)

        _terminate_actors(world_desc=world_desc)
        _log_instance_status()
        return _get_log()

    except Exception:
        _log(f"INTERNAL EXCEPTION:\n{traceback.format_exc()}")
        return _get_log()


#
# High-level helpers
#

def _initialize_instance():
    global _instance, _game, _editor, _world_descs, _exec_namespace

    config = spear.get_config()
    config.defrost()
    config.SPEAR.INSTANCE.CLIENT_INTERNAL_TIMEOUT_SECONDS = 3600.0
    config.freeze()
    _instance = spear.Instance(config=config)
    _log("Instance created.")

    world_descs = _instance.world_registry_service.get_world_descs()
    editor_available = any(desc.is_editor_world for desc in world_descs.values())
    game_available = any(desc.is_game_world and desc.is_playing for desc in world_descs.values())

    if not editor_available and not game_available:
        _log("ERROR: No editor or game available.")
        return False

    # detect world changes and invalidate agent variables if needed
    if _world_descs is not None:
        world_changed = False
        if set(world_descs.keys()) != set(_world_descs.keys()):
            world_changed = True
        else:
            for name in world_descs:
                if world_descs[name].world_id != _world_descs[name].world_id:
                    world_changed = True
                    break
        if world_changed:
            _log("World state changed since last call. Clearing agent variables.")
            _exec_namespace.clear()
    _world_descs = world_descs

    if game_available:
        _game = _instance.get_game()
        _log("Game obtained.")

    if editor_available:
        _editor = _instance.get_editor()
        with _instance.begin_frame():
            _editor.python_service.execute_string(string="import math", execution_scope="Public")
        with _instance.end_frame():
            pass
        _log("Editor obtained.")

    _initialize_persistent_actors()

    _exec_namespace["math"] = math
    _exec_namespace["np"] = np
    _exec_namespace["spear"] = spear

    _exec_namespace["instance"] = _instance
    _exec_namespace["game"] = _game
    _exec_namespace["editor"] = _editor

    return True


def _terminate_instance():
    global _instance, _game, _editor, _world_descs, _exec_namespace

    _exec_namespace.pop("math", None)
    _exec_namespace.pop("np", None)
    _exec_namespace.pop("spear", None)

    _exec_namespace.pop("instance", None)
    _exec_namespace.pop("game", None)
    _exec_namespace.pop("editor", None)

    _terminate_persistent_actors()

    _world_descs = None

    if _game is not None:
        _game = None
        _log("Game terminated.")

    if _editor is not None:
        _editor = None
        _log("Editor terminated.")

    _instance = None
    _log("Instance terminated.")


def _initialize_persistent_actors():

    if _game is not None:
        world_scoped_services = _game
    elif _editor is not None:
        world_scoped_services = _editor
    else:
        _log("ERROR: No editor or game available.")
        return

    with _instance.begin_frame():
        existing = world_scoped_services.unreal_service.find_actors_by_class(uclass="ASpStableNameManager")
        if len(existing) == 0:
            stable_name_manager = world_scoped_services.unreal_service.spawn_actor(uclass="ASpStableNameManager")
            world_scoped_services.unreal_service.set_stable_name_for_actor(actor=stable_name_manager, stable_name=_STABLE_NAME_STABLE_NAME_MANAGER)
            if _instance.engine_globals_service.is_with_editor():
                stable_name_manager.SetFolderPath(NewFolderPath=_EDITOR_FOLDER_PATH)
                stable_name_manager.SetActorLabel(NewActorLabel=_STABLE_NAME_STABLE_NAME_MANAGER)
            _log("Spawned ASpStableNameManager.")
        else:
            _log("ASpStableNameManager already exists.")
    with _instance.end_frame():
        pass


def _terminate_persistent_actors():

    if _game is not None:
        world_scoped_services = _game
    elif _editor is not None:
        world_scoped_services = _editor
    else:
        _log("ERROR: No editor or game available.")
        return

    with _instance.begin_frame():
        for actor in world_scoped_services.unreal_service.find_actors_by_name(actor_name=_STABLE_NAME_STABLE_NAME_MANAGER, uclass="AActor"):
            world_scoped_services.unreal_service.destroy_actor(actor=actor)
            _log("Destroyed ASpStableNameManager.")
    with _instance.end_frame():
        pass


def _initialize_actors():

    if _game is not None:
        world_scoped_services = _game
        world_type = "game"
    elif _editor is not None:
        world_scoped_services = _editor
        world_type = "editor"
    else:
        _log("ERROR: No editor or game available.")
        return False, None

    world_desc = {"world_scoped_services": world_scoped_services, "world_type": world_type, "proxy_component_manager": None, "camera_sensor": None, "camera_components": {}}

    success = True
    with _instance.begin_frame():

        # find and remove leaked actors

        is_with_editor = _instance.engine_globals_service.is_with_editor()

        if is_with_editor:
            stable_names = [f"{_EDITOR_FOLDER_PATH}/{name}" for name in [_STABLE_NAME_CAMERA_SENSOR, _STABLE_NAME_PROXY_COMPONENT_MANAGER]]
        else:
            stable_names = [_STABLE_NAME_CAMERA_SENSOR, _STABLE_NAME_PROXY_COMPONENT_MANAGER]

        for stable_name in stable_names:
            for actor in world_scoped_services.unreal_service.find_actors_by_name(actor_name=stable_name, uclass="AActor"):
                world_scoped_services.unreal_service.destroy_actor(actor=actor)

        # get viewport and camera info
        if world_type == "game":
            engine = _instance.engine_globals_service.get_engine()
            game_viewport_client = engine.GameViewport.get()
            sp_game_viewport = world_scoped_services.get_unreal_object(uclass="USpGameViewportClient")
            viewport_size = sp_game_viewport.GetViewportSize(GameViewportClient=game_viewport_client, as_dict=True)["ViewportSize"]
            viewport_size_x = viewport_size["x"]
            viewport_size_y = viewport_size["y"]

            gameplay_statics = world_scoped_services.get_unreal_object(uclass="UGameplayStatics")
            player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)
            pov = player_controller.PlayerCameraManager.ViewTarget.POV.get()
            camera_location = pov["location"]
            camera_rotation = pov["rotation"]
            is_perspective = pov["projectionMode"] == "Perspective"
            is_orthographic = pov["projectionMode"] == "Orthographic"

            if is_perspective + is_orthographic != 1:
                _log("ERROR: View target is neither Perspective nor Orthographic.")
                success = False

            if is_perspective:
                fov_degrees = pov["fOV"]
                aspect_ratio = pov["aspectRatio"]
            elif is_orthographic:
                ortho_width = pov["orthoWidth"]
            else:
                pass # handled above

        elif world_type == "editor":
            sp_editor_engine = world_scoped_services.get_unreal_object(uclass="USpEditorEngine")
            viewport_client_descs = sp_editor_engine.GetLevelViewportClients(as_dict=True)["ReturnValue"]
            editing_descs = [desc for desc in viewport_client_descs if desc["bIsCurrentLevelEditing"]]

            if len(editing_descs) != 1:
                _log(f"ERROR: Expected exactly one editor viewport with bIsCurrentLevelEditing, found {len(editing_descs)}.")
                success = False
            else:
                client = editing_descs[0]
                viewport_size_x = client["viewportSize"]["x"]
                viewport_size_y = client["viewportSize"]["y"]
                camera_location = client["cameraLocation"]
                camera_rotation = client["cameraRotation"]
                is_perspective = client["bIsPerspective"]
                is_orthographic = not is_perspective

                if is_perspective:
                    fov_degrees = client["viewFOV"]
                    aspect_ratio = client["aspectRatio"]
                elif is_orthographic:
                    ortho_width = client["orthoZoom"]
                else:
                    pass # unreachable because is_orthographic is defined as not is_perspective

        if success:

            # compute image dims
            if viewport_size_x >= viewport_size_y:
                image_width = _MAX_IMAGE_DIM
                image_height = int(_MAX_IMAGE_DIM * viewport_size_y / viewport_size_x)
            else:
                image_height = _MAX_IMAGE_DIM
                image_width = int(_MAX_IMAGE_DIM * viewport_size_x / viewport_size_y)

            # spawn proxy component manager and camera sensor

            world_desc["proxy_component_manager"] = world_scoped_services.unreal_service.spawn_actor(uclass="ASpObjectIdsProxyComponentManager")
            if is_with_editor:
                world_desc["proxy_component_manager"].SetFolderPath(NewFolderPath=_EDITOR_FOLDER_PATH)
                world_desc["proxy_component_manager"].SetActorLabel(NewActorLabel=_STABLE_NAME_PROXY_COMPONENT_MANAGER)
            else:
                world_scoped_services.unreal_service.set_stable_name_for_actor(actor=world_desc["proxy_component_manager"], stable_name=_STABLE_NAME_PROXY_COMPONENT_MANAGER)
            world_desc["proxy_component_manager"].Initialize()

            bp_cam_uclass = world_scoped_services.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
            world_desc["camera_sensor"] = world_scoped_services.unreal_service.spawn_actor(uclass=bp_cam_uclass)
            if is_with_editor:
                world_desc["camera_sensor"].SetFolderPath(NewFolderPath=_EDITOR_FOLDER_PATH)
                world_desc["camera_sensor"].SetActorLabel(NewActorLabel=_STABLE_NAME_CAMERA_SENSOR)
            else:
                world_scoped_services.unreal_service.set_stable_name_for_actor(actor=world_desc["camera_sensor"], stable_name=_STABLE_NAME_CAMERA_SENSOR)

            # get camera components
            world_desc["camera_components"] = {
                "final_tone_curve_hdr": world_scoped_services.unreal_service.get_component_by_name(actor=world_desc["camera_sensor"], component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D"),
                "object_ids_uint8": world_scoped_services.unreal_service.get_component_by_name(actor=world_desc["camera_sensor"], component_name="DefaultSceneRoot.object_ids_uint8_", uclass="USpSceneCaptureComponent2D"),
                "sp_depth_meters": world_scoped_services.unreal_service.get_component_by_name(actor=world_desc["camera_sensor"], component_name="DefaultSceneRoot.sp_depth_meters_", uclass="USpSceneCaptureComponent2D"),
                "sp_world_position": world_scoped_services.unreal_service.get_component_by_name(actor=world_desc["camera_sensor"], component_name="DefaultSceneRoot.sp_world_position_", uclass="USpSceneCaptureComponent2D"),
                "world_normal": world_scoped_services.unreal_service.get_component_by_name(actor=world_desc["camera_sensor"], component_name="DefaultSceneRoot.world_normal_", uclass="USpSceneCaptureComponent2D")}

            # set camera location and rotation
            world_desc["camera_sensor"].K2_SetActorLocation(NewLocation=camera_location)
            world_desc["camera_sensor"].K2_SetActorRotation(NewRotation=camera_rotation)

            # initialize camera components
            for component in world_desc["camera_components"].values():

                component.Width = image_width
                component.Height = image_height

                if is_perspective:
                    viewport_aspect_ratio = viewport_size_x / viewport_size_y
                    fov = fov_degrees * math.pi / 180.0
                    half_fov = fov / 2.0
                    half_fov_adjusted = math.atan(math.tan(half_fov) * viewport_aspect_ratio / aspect_ratio)
                    fov_adjusted_degrees = half_fov_adjusted * 2.0 * 180.0 / math.pi
                    component.ProjectionType = "Perspective"
                    component.FOVAngle = fov_adjusted_degrees
                elif is_orthographic:
                    component.ProjectionType = "Orthographic"
                    component.OrthoWidth = ortho_width
                else:
                    pass # handled above

                component.Initialize()
                component.initialize_sp_funcs()

    with _instance.end_frame():
        pass

    if success:
        _instance.step(num_frames=2)
        _exec_namespace["camera_components"] = world_desc["camera_components"]
        _exec_namespace["proxy_component_manager"] = world_desc["proxy_component_manager"]
        _log(f"Actors initialized: {image_width}x{image_height} camera, {'perspective' if is_perspective else 'orthographic'} projection.")

    return success, world_desc


def _terminate_actors(world_desc):

    world_scoped_services = world_desc["world_scoped_services"]

    with _instance.begin_frame():
        for component in world_desc["camera_components"].values():
            component.terminate_sp_funcs()
            component.Terminate()

        if world_desc["camera_sensor"] is not None:
            world_scoped_services.unreal_service.destroy_actor(actor=world_desc["camera_sensor"])

        if world_desc["proxy_component_manager"] is not None:
            world_desc["proxy_component_manager"].Terminate()
            world_scoped_services.unreal_service.destroy_actor(actor=world_desc["proxy_component_manager"])

    with _instance.end_frame():
        pass

    _log("Actors terminated.")


def _log_instance_status():

    _log(f"Instance: {'not ' if _instance is None else ''}available.")

    if _instance is None:
        return

    frame_state = _instance._engine_service._frame_state
    if frame_state != "idle":
        _log(f'ERROR: Unexpected frame state: "{frame_state}".')

    if _game is None and _editor is None:
        _log("ERROR: No editor or game available.")
    else:
        _log(f"Game: {'not ' if _game is None else ''}available.")
        _log(f"Editor: {'not ' if _editor is None else ''}available.")


def _validate_code(code):

    try:
        tree = ast.parse(code)
    except SyntaxError as e:
        _log(f"ERROR: SyntaxError: {e}")
        return False

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            _log(f"ERROR: import statement not allowed: import {', '.join(alias.name for alias in node.names)}")
            return False
        if isinstance(node, ast.ImportFrom):
            _log(f"ERROR: import statement not allowed: from {node.module} import ...")
            return False
        if isinstance(node, ast.Name) and node.id in _BANNED_NAMES:
            _log(f"ERROR: name not allowed: {node.id}")
            return False
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Name) and func.id in _BANNED_CALLS:
                _log(f"ERROR: call not allowed: {func.id}()")
                return False
            if isinstance(func, ast.Attribute) and func.attr in _BANNED_CALLS:
                _log(f"ERROR: call not allowed: .{func.attr}()")
                return False
        if isinstance(node, ast.Attribute) and node.attr in _BANNED_ATTRS:
            _log(f"ERROR: attribute access not allowed: .{node.attr}")
            return False

    return True


def _save_images(variable_names):

    os.makedirs(_SAVE_DIR, exist_ok=True)

    for name in variable_names:
        if name not in _exec_namespace:
            _log(f"ERROR: {name} not found in namespace")
            return

        obj = _exec_namespace[name]
        path = os.path.realpath(os.path.join(_SAVE_DIR, f"{name}.png"))

        if not isinstance(obj, np.ndarray):
            _log(f"ERROR: {name} is not a numpy array (type={type(obj).__name__})")
            return
        elif obj.dtype != np.uint8:
            _log(f"ERROR: {name} must be uint8 (dtype={obj.dtype})")
            return
        elif not obj.flags.owndata:
            _log(f"ERROR: {name} does not own its data. Results from read_pixels() are volatile - always deep copy with .copy() before storing (e.g., data = result['arrays']['data'].copy()).")
            return

        if obj.ndim == 2 or (obj.ndim == 3 and obj.shape[2] in (1, 3)):
            cv2.imwrite(path, obj)
            _log(f"Saved image: {path}")
        elif obj.ndim == 3 and obj.shape[2] == 4:
            img = obj.copy()
            img[:,:,3] = 255
            cv2.imwrite(path, img)
            _log(f"Saved image: {path}")
        else:
            _log(f"ERROR: {name} has an unsupported shape (shape={obj.shape})")


#
# Main
#

if __name__ == "__main__":
    _log("Server starting...")
    _mcp.run(transport="stdio")
