#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import ast
import cv2
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


#
# Global state
#

_mcp = mcp.server.fastmcp.FastMCP("spear")

_instance = None
_game = None
_editor = None
_scope = None
_scope_type = None  # "game" or "editor"
_exec_namespace = {}


#
# MCP tools
#

@_mcp.tool()
def get_status():
    _ensure_initialized()
    return "\n".join(_get_status_lines())


@_mcp.tool()
def execute_code(code, save_images=None):

    is_valid, reason = _validate_code(code=code)
    if not is_valid:
        return f"Code rejected by validator: {reason}"

    init_messages = _ensure_initialized()
    if _scope is None:
        return "\n\n".join(init_messages + ["No scope available."])

    logs = []
    error_text = None

    warnings = _update_camera()

    frame_state = _instance._engine_service._frame_state
    assert frame_state == "idle"

    log_func = lambda msg: logs.append(msg)
    spear.register_log_func(func=log_func)
    try:
        exec(code, _exec_namespace)
    except Exception:
        error_text = traceback.format_exc()
    finally:
        spear.unregister_log_func(func=log_func)

    frame_state = _instance._engine_service._frame_state
    frame_state_error = None
    if frame_state == "idle":
        pass
    elif frame_state == "executing_frame":
        with _instance.end_frame():
            pass
        frame_state_error = "The code has a begin_frame block without a matching end_frame block."
    else:
        frame_state_error = f'Unexpected frame state after executing user code: "{frame_state}".'

    if frame_state_error is not None:
        if error_text is None:
            error_text = frame_state_error
        else:
            error_text += "\n\n" + frame_state_error

    # save images if requested
    image_results = []
    if save_images is not None and error_text is None:
        image_results = _save_images(variable_names=save_images)

    return _format_results(init_messages=init_messages, warnings=warnings, logs=logs, error_text=error_text, image_results=image_results)


@_mcp.tool()
def execute_editor_code(code):

    is_valid, reason = _validate_code(code=code)
    if not is_valid:
        return f"Code rejected by validator: {reason}"

    init_messages = _ensure_initialized()
    if _editor is None:
        return "\n\n".join(init_messages + ["Editor services not available."])

    logs = []
    error_text = None

    warnings = _update_camera()

    log_func = lambda msg: logs.append(msg)
    spear.register_log_func(func=log_func)
    try:
        with _instance.begin_frame():
            try:
                _editor.python_service.execute_string(string=code, execution_scope="Public")
            except RuntimeError:
                error_text = traceback.format_exc()
        with _instance.end_frame():
            pass
    finally:
        spear.unregister_log_func(func=log_func)

    return _format_results(init_messages=init_messages, warnings=warnings, logs=logs, error_text=error_text, image_results=[])


@_mcp.tool()
def execute_editor_code_across_frames(code):

    is_valid, reason = _validate_code(code=code)
    if not is_valid:
        return f"Code rejected by validator: {reason}"

    init_messages = _ensure_initialized()
    if _editor is None:
        return "\n\n".join(init_messages + ["Editor services not available."])

    logs = []
    error_text = None

    warnings = _update_camera()

    log_func = lambda msg: logs.append(msg)
    spear.register_log_func(func=log_func)
    try:
        try:
            _editor.python_service.execute_string_across_frames(string=code, execution_scope="Public")
        except RuntimeError:
            error_text = traceback.format_exc()
    finally:
        spear.unregister_log_func(func=log_func)

    return _format_results(init_messages=init_messages, warnings=warnings, logs=logs, error_text=error_text, image_results=[])


#
# High-level helpers
#

def _ensure_initialized():
    global _instance, _game, _editor, _scope, _scope_type, _exec_namespace

    messages = []

    if _instance is None:
        config = spear.get_config()
        config.defrost()
        config.SPEAR.INSTANCE.CLIENT_INTERNAL_TIMEOUT_SECONDS = 3600.0
        config.freeze()
        _instance = spear.Instance(config=config)

    # determine availability
    is_with_editor = _instance.engine_globals_service.is_with_editor()
    command_line = _instance.engine_globals_service.get_command_line()
    game_available = _instance._game.initialize_game_world_service.is_initialized()
    editor_available = is_with_editor and " -game " not in command_line

    # obtain services
    if game_available and _game is None:
        _game = _instance.get_game()
        messages.append("Game services obtained.")
    elif not game_available and _game is not None:
        _game = None
        messages.append("Game services no longer available, cleared.")

    if editor_available and _editor is None:
        _editor = _instance.get_editor()
        messages.append("Editor services obtained.")
    elif not editor_available and _editor is not None:
        _editor = None
        messages.append("Editor services no longer available, cleared.")

    # determine which scope type should own the camera and proxy manager
    if game_available:
        desired_type = "game"
    elif editor_available:
        desired_type = "editor"
    else:
        desired_type = None

    # tear down if scope type changed
    if _scope is not None and _scope_type != desired_type:
        previous_type = _scope_type
        with _instance.begin_frame():
            pass
        with _instance.end_frame():
            _terminate_scope_gt(scope=_scope)
        _scope = None
        _scope_type = None
        messages.append(f"Camera and proxy manager torn down (was {previous_type}).")

    # set up new scope if needed
    if desired_type is not None and _scope is None:
        _scope = {"scoped_services": None, "proxy_component_manager": None, "camera_sensor": None, "camera_components": {}}
        _scope_type = desired_type

        if _scope_type == "game":
            _scope["scoped_services"] = _game
        elif _scope_type == "editor":
            _scope["scoped_services"] = _editor
        else:
            assert False

        with _instance.begin_frame():
            num_destroyed = _cleanup_leaked_actors_gt(scoped_services=_scope["scoped_services"])
            if num_destroyed > 0:
                messages.append(f"Cleaned up {num_destroyed} leaked actor(s) from a previous session.")
            if _scope_type == "game":
                engine = _scope["scoped_services"].engine_globals_service.get_engine()
                game_viewport_client = engine.GameViewport.get()
                sp_game_viewport = _scope["scoped_services"].get_unreal_object(uclass="USpGameViewportClient")
                viewport_size = sp_game_viewport.GetViewportSize(GameViewportClient=game_viewport_client, as_dict=True)["ViewportSize"]
                viewport_size_x = viewport_size["x"]
                viewport_size_y = viewport_size["y"]
            elif _scope_type == "editor":
                _editor.python_service.execute_string(string="import math", execution_scope="Public")
                sp_editor_engine = _scope["scoped_services"].get_unreal_object(uclass="USpEditorEngine")
                viewport_client_descs = sp_editor_engine.GetLevelViewportClients(as_dict=True)["ReturnValue"]
                perspective_descs = [desc for desc in viewport_client_descs if desc["bIsPerspective"]]
                if len(perspective_descs) == 1:
                    viewport_size_x = perspective_descs[0]["viewportSize"]["x"]
                    viewport_size_y = perspective_descs[0]["viewportSize"]["y"]
                else:
                    viewport_size_x = _MAX_IMAGE_DIM
                    viewport_size_y = _MAX_IMAGE_DIM
            else:
                assert False
            img_w, img_h = _compute_image_dims_for_rendering(viewport_size_x=viewport_size_x, viewport_size_y=viewport_size_y)
            _initialize_scope_gt(scope=_scope, img_w=img_w, img_h=img_h)
            if _scope_type == "game":
                _update_game_camera_gt()
            elif _scope_type == "editor":
                warning = _update_editor_camera_gt()
                if warning is not None:
                    messages.append(warning)
            else:
                assert False
        with _instance.end_frame():
            pass

        _instance.flush(num_frames=2)
        messages.append(f"{_scope_type.capitalize()} camera ({img_w}x{img_h}) and proxy component manager initialized.")

    # update exec namespace (preserve user variables)
    _exec_namespace["instance"] = _instance
    _exec_namespace["spear"] = spear
    _exec_namespace["np"] = np
    _exec_namespace["math"] = math

    if _game is not None:
        _exec_namespace["game"] = _game
    if _editor is not None:
        _exec_namespace["editor"] = _editor
    if _scope is not None:
        _exec_namespace["camera_components"] = _scope["camera_components"]
        _exec_namespace["proxy_component_manager"] = _scope["proxy_component_manager"]

    return messages

def _get_status_lines():
    lines = []
    if _game is not None:
        lines.append("Game scope: available.")
    else:
        lines.append("Game scope: not available.")
    if _editor is not None:
        lines.append("Editor scope: available.")
    else:
        lines.append("Editor scope: not available.")
    if _scope is not None:
        lines.append(f"Active scope: {_scope_type}.")
    else:
        lines.append("Active scope: none.")
    return lines

def _validate_code(code):
    try:
        tree = ast.parse(code)
    except SyntaxError as e:
        return False, f"SyntaxError: {e}"

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            return False, f"import statement not allowed: import {', '.join(alias.name for alias in node.names)}"
        if isinstance(node, ast.ImportFrom):
            return False, f"import statement not allowed: from {node.module} import ..."
        if isinstance(node, ast.Name) and node.id in _BANNED_NAMES:
            return False, f"name not allowed: {node.id}"
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Name) and func.id in _BANNED_CALLS:
                return False, f"call not allowed: {func.id}()"
            if isinstance(func, ast.Attribute) and func.attr in _BANNED_CALLS:
                return False, f"call not allowed: .{func.attr}()"
        if isinstance(node, ast.Attribute) and node.attr in _BANNED_ATTRS:
            return False, f"attribute access not allowed: .{node.attr}"

    return True, ""

def _update_camera():

    warnings = []

    with _instance.begin_frame():
        if _scope_type == "game":
            _update_game_camera_gt()
        elif _scope_type == "editor":
            warning = _update_editor_camera_gt()
            if warning is not None:
                warnings.append(warning)
        else:
            assert False
    with _instance.end_frame():
        pass

    return warnings

def _format_results(init_messages, warnings, logs, error_text, image_results):
    lines = []
    if init_messages:
        lines.append("\n".join(init_messages))
    if warnings:
        lines.append("Warnings:\n" + "\n".join(warnings))
    if logs:
        lines.append("Logs:\n" + "\n".join(logs))
    if error_text:
        lines.append("Exception:\n" + error_text)
    if image_results:
        saved = [r for r in image_results if r.startswith("/")]
        errors = [r for r in image_results if not r.startswith("/")]
        if saved:
            lines.append("Saved:\n" + "\n".join(saved))
        if errors:
            lines.append("Image errors:\n" + "\n".join(errors))
    if not lines:
        lines.append("Code executed successfully with no output.")
    lines.append("Status:\n" + "\n".join(_get_status_lines()))
    return "\n\n".join(lines)

def _save_images(variable_names):
    os.makedirs(_SAVE_DIR, exist_ok=True)
    saved = []
    errors = []
    for name in variable_names:
        if name not in _exec_namespace:
            errors.append(f"{name}: not found in namespace")
            continue
        obj = _exec_namespace[name]
        path = os.path.realpath(os.path.join(_SAVE_DIR, f"{name}.png"))
        if not isinstance(obj, np.ndarray):
            errors.append(f"{name}: not a numpy array (type={type(obj).__name__})")
        elif obj.dtype != np.uint8:
            errors.append(f"{name}: must be uint8 (got {obj.dtype})")
        elif obj.ndim == 2 or (obj.ndim == 3 and obj.shape[2] in (1, 3)):
            cv2.imwrite(path, obj)
            saved.append(path)
        elif obj.ndim == 3 and obj.shape[2] == 4:
            img = obj.copy()
            img[:,:,3] = 255
            cv2.imwrite(path, img)
            saved.append(path)
        else:
            errors.append(f"{name}: unsupported array shape {obj.shape}")
    return saved + errors

def _compute_image_dims_for_rendering(viewport_size_x, viewport_size_y):

    if viewport_size_x >= viewport_size_y:
        img_w = _MAX_IMAGE_DIM
        img_h = int(_MAX_IMAGE_DIM * viewport_size_y / viewport_size_x)
    else:
        img_h = _MAX_IMAGE_DIM
        img_w = int(_MAX_IMAGE_DIM * viewport_size_x / viewport_size_y)
    return img_w, img_h


#
# Game-thread helpers
#

def _initialize_scope_gt(scope, img_w, img_h):

    scoped_services = scope["scoped_services"]

    scope["proxy_component_manager"] = scoped_services.unreal_service.spawn_actor(uclass="ASpObjectIdsProxyComponentManager")
    scoped_services.unreal_service.set_stable_name_for_actor(actor=scope["proxy_component_manager"], stable_name=_STABLE_NAME_PROXY_COMPONENT_MANAGER)
    scope["proxy_component_manager"].Initialize()

    bp_cam_uclass = scoped_services.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
    scope["camera_sensor"] = scoped_services.unreal_service.spawn_actor(uclass=bp_cam_uclass)

    scoped_services.unreal_service.set_stable_name_for_actor(actor=scope["camera_sensor"], stable_name=_STABLE_NAME_CAMERA_SENSOR)

    scope["camera_components"] = {
        "final_tone_curve_hdr": scoped_services.unreal_service.get_component_by_name(actor=scope["camera_sensor"], component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D"),
        "object_ids_uint8": scoped_services.unreal_service.get_component_by_name(actor=scope["camera_sensor"], component_name="DefaultSceneRoot.object_ids_uint8_", uclass="USpSceneCaptureComponent2D"),
        "sp_depth_meters": scoped_services.unreal_service.get_component_by_name(actor=scope["camera_sensor"], component_name="DefaultSceneRoot.sp_depth_meters_", uclass="USpSceneCaptureComponent2D"),
        "sp_world_position": scoped_services.unreal_service.get_component_by_name(actor=scope["camera_sensor"], component_name="DefaultSceneRoot.sp_world_position_", uclass="USpSceneCaptureComponent2D"),
        "world_normal": scoped_services.unreal_service.get_component_by_name(actor=scope["camera_sensor"], component_name="DefaultSceneRoot.world_normal_", uclass="USpSceneCaptureComponent2D"),
    }

    for comp in scope["camera_components"].values():
        comp.Width = img_w
        comp.Height = img_h
        comp.Initialize()
        comp.initialize_sp_funcs()

def _terminate_scope_gt(scope):

    scoped_services = scope["scoped_services"]

    for comp in scope["camera_components"].values():
        comp.terminate_sp_funcs()
        comp.Terminate()
    scoped_services.unreal_service.destroy_actor(actor=scope["camera_sensor"])
    scope["proxy_component_manager"].Terminate()
    scoped_services.unreal_service.destroy_actor(actor=scope["proxy_component_manager"])

def _cleanup_leaked_actors_gt(scoped_services):

    num_destroyed = 0
    for stable_name in [_STABLE_NAME_CAMERA_SENSOR, _STABLE_NAME_PROXY_COMPONENT_MANAGER]:
        leaked_actors = scoped_services.unreal_service.find_actors_by_name(actor_name=stable_name, uclass="AActor")
        for actor in leaked_actors:
            scoped_services.unreal_service.destroy_actor(actor=actor)
            num_destroyed += 1
    return num_destroyed

def _update_game_camera_gt():

    scoped_services = _scope["scoped_services"]

    gameplay_statics = scoped_services.get_unreal_object(uclass="UGameplayStatics")
    player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)
    player_camera_manager = player_controller.PlayerCameraManager.get()
    pov = player_camera_manager.ViewTarget.POV.get()

    engine = scoped_services.engine_globals_service.get_engine()
    game_viewport_client = engine.GameViewport.get()
    sp_game_viewport = scoped_services.get_unreal_object(uclass="USpGameViewportClient")
    viewport_size = sp_game_viewport.GetViewportSize(GameViewportClient=game_viewport_client, as_dict=True)["ViewportSize"]
    viewport_size_x = viewport_size["x"]
    viewport_size_y = viewport_size["y"]
    viewport_aspect_ratio = viewport_size_x / viewport_size_y

    fov = pov["fOV"] * math.pi / 180.0
    half_fov = fov / 2.0
    half_fov_adjusted = math.atan(math.tan(half_fov) * viewport_aspect_ratio / pov["aspectRatio"])
    fov_adjusted_degrees = half_fov_adjusted * 2.0 * 180.0 / math.pi

    _scope["camera_sensor"].K2_SetActorLocation(NewLocation=pov["location"])
    _scope["camera_sensor"].K2_SetActorRotation(NewRotation=pov["rotation"])

    for comp in _scope["camera_components"].values():
        comp.FOVAngle = fov_adjusted_degrees

def _update_editor_camera_gt():

    scoped_services = _scope["scoped_services"]

    sp_editor_engine = scoped_services.get_unreal_object(uclass="USpEditorEngine")
    viewport_client_descs = sp_editor_engine.GetLevelViewportClients(as_dict=True)["ReturnValue"]
    perspective_descs = [desc for desc in viewport_client_descs if desc["bIsPerspective"]]

    if len(perspective_descs) != 1:
        return "WARNING: expected exactly one perspective editor viewport, editor camera not updated."

    client = perspective_descs[0]

    viewport_size_x = client["viewportSize"]["x"]
    viewport_size_y = client["viewportSize"]["y"]
    viewport_aspect_ratio = viewport_size_x / viewport_size_y

    fov = client["viewFOV"] * math.pi / 180.0
    half_fov = fov / 2.0
    half_fov_adjusted = math.atan(math.tan(half_fov) * viewport_aspect_ratio / client["aspectRatio"])
    fov_adjusted_degrees = half_fov_adjusted * 2.0 * 180.0 / math.pi

    root = scoped_services.unreal_service.get_component_by_name(actor=_scope["camera_sensor"], component_name="DefaultSceneRoot", uclass="USceneComponent")
    root.K2_SetWorldLocation(NewLocation=client["cameraLocation"])
    root.K2_SetWorldRotation(NewRotation=client["cameraRotation"])

    for comp in _scope["camera_components"].values():
        comp.FOVAngle = fov_adjusted_degrees

#
# Main
#

if __name__ == "__main__":
    print("[run_mcp_server.py] Server starting...", file=sys.stderr, flush=True) # can't use spear.log because we're writing to sys.stderr
    _mcp.run(transport="stdio")
