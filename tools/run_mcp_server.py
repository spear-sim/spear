#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import ast
import colorsys
import inspect
import math
import matplotlib.pyplot as plt
import mcp
import numpy as np
import os
import spear
import sys
import traceback


#
# Constants
#

_BANNED_ATTRS = {
    "__bases__", "__builtins__", "__class__", "__code__", "__dict__", "__func__", "__globals__", "__locals__", "__subclasses__", "__wrapped__"}

_BANNED_CALLS = {
    "__import__", "compile", "delattr", "eval", "evaluate_expression", "exec", "execute_file", "execute_file_across_frames", "get_editor", "get_game",
    "getattr", "open", "OpenLevel", "save", "savez", "savez_compressed", "savetxt", "setattr", "tofile"}

_BANNED_NAMES = {
    "__builtins__", "__import__", "builtins", "importlib", "os", "pathlib", "shutil", "socket", "subprocess", "sys"}

_SAVE_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), "tmp", "spear-mcp"))
_MAX_IMAGE_DIM = 256

_STABLE_NAME_CAMERA_SENSOR = "__SP_CAMERA_SENSOR__"
_M_HYPERSIM_CAMERA_FROM_UNREAL_CAMERA = np.matrix([[0, 1, 0], [0, 0, 1], [-1, 0, 0]], dtype=np.float32)

_EXEC_NAMESPACE_DESCS = {
    "math": "Python math module",
    "np": "NumPy",
    "spear": "SPEAR Python module",
    "instance": "spear.Instance connected to the running engine",
    "game": "game world scoped services (None if no game)",
    "editor": "editor world scoped services (None if no editor)",
    "viewport_desc": "dict with camera pose, FOV, viewport size",
    "before_execute": "dict of viewport data captured before agent code runs (see inner keys below, identical to after_execute when calling the 'get_viewport_data' tool)",
    "after_execute": "dict of viewport data captured after agent code runs (see inner keys below, identical to before_execute when calling the 'get_viewport_data' tool)"}

_RENDER_DATA_DESCS = {
    "final_tone_curve_hdr": "RGB image, uint8, (H,W,3)",
    "depth_meters": "depth in meters, float32, (H,W)",
    "world_position": "XYZ world coordinates, float32, (H,W,3)",
    "world_normal": "surface normals in world space, float32, (H,W,3)",
    "camera_normal": "surface normals in Hypersim camera space, float32, (H,W,3)",
    "camera_position": "positions in Hypersim camera space, float32, (H,W,3)",
    "segmentation_id_image": "per-pixel index into segmentation_id_descs, int32, (H,W)",
    "segmentation_id_descs": "list of dicts with actor/component/material handles and names"}

_SAVED_IMAGE_DESCS = {
    "final_tone_curve_hdr.png": "RGB image",
    "depth_meters.png": "depth shifted by min, divided by K = min(span, 7.5) meters",
    "camera_normal.png": "surface normals in Hypersim camera space, (1+n)/2",
    "camera_position.png": "positions in Hypersim camera space, per-channel median centered at 0.5, uniform K = min(2 * max_channel_abs_dev, 750)",
    "segmentation_colors.png": "random colors per segmentation ID"}


#
# Global state
#

_mcp = mcp.server.fastmcp.FastMCP("spear")

_instance = None
_game = None
_editor = None
_world_descs = None
_exec_namespace = {}
_before_execute_populated = False
_after_execute_populated = False


#
# MCP tools
#

@_mcp.tool()
def get_viewport_data():
    global _before_execute_populated, _after_execute_populated
    _before_execute_populated = False
    _after_execute_populated = False

    try:
        success, world_desc = _tool_prefix()
        if not success:
            _log_status()
            return _get_log()

        _tool_suffix(world_desc=world_desc)
        return _get_log()

    except Exception:
        _log(f"INTERNAL EXCEPTION:\n{traceback.format_exc()}")
        return _get_log()


@_mcp.tool()
def execute_code(code):
    global _before_execute_populated, _after_execute_populated
    _before_execute_populated = False
    _after_execute_populated = False

    try:
        success = _validate_code(code=code)
        if not success:
            _log_status()
            return _get_log()

        success, world_desc = _tool_prefix()
        if not success:
            _log_status()
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

        _tool_suffix(world_desc=world_desc)
        return _get_log()

    except Exception:
        _log(f"INTERNAL EXCEPTION:\n{traceback.format_exc()}")
        return _get_log()


@_mcp.tool()
def execute_editor_code(code):
    global _before_execute_populated, _after_execute_populated
    _before_execute_populated = False
    _after_execute_populated = False

    try:
        success = _validate_code(code=code)
        if not success:
            _log_status()
            return _get_log()

        success, world_desc = _tool_prefix()
        if not success:
            _log_status()
            return _get_log()

        if _editor is None:
            _log("ERROR: Editor not available.")
            _tool_suffix(world_desc=world_desc)
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

        _tool_suffix(world_desc=world_desc)
        return _get_log()

    except Exception:
        _log(f"INTERNAL EXCEPTION:\n{traceback.format_exc()}")
        return _get_log()


#
# Helper functionss
#

def _tool_prefix():
    success = _initialize_instance()
    if not success:
        return False, None

    _initialize_namespace()

    success, world_desc = _initialize_actors()
    if not success:
        return False, None

    _get_image_data(world_desc=world_desc, key="before_execute")
    _save_images(world_desc=world_desc, key="before_execute")
    _update_exec_namespace(world_desc=world_desc, key="before_execute")

    return True, world_desc


def _tool_suffix(world_desc):
    _get_image_data(world_desc=world_desc, key="after_execute")
    _save_images(world_desc=world_desc, key="after_execute")
    _update_exec_namespace(world_desc=world_desc, key="after_execute")

    _terminate_actors(world_desc=world_desc)
    _log_status()


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
        _log("Editor obtained.")

    return True


def _initialize_namespace():

    _exec_namespace["math"] = math
    _exec_namespace["np"] = np
    _exec_namespace["spear"] = spear
    _exec_namespace["instance"] = _instance
    _exec_namespace["game"] = _game
    _exec_namespace["editor"] = _editor

    if _editor is not None:
        with _instance.begin_frame():
            _editor.python_service.execute_string(string="import math", execution_scope="Public")
        with _instance.end_frame():
            pass


def _initialize_actors():

    if _game is not None:
        world_scoped_services = _game
    elif _editor is not None:
        world_scoped_services = _editor
    else:
        _log("ERROR: No editor or game available.")
        return False, None

    world_desc = {"world_scoped_services": world_scoped_services, "viewport_desc": None, "camera_sensor": None, "camera_components": {}}

    with _instance.begin_frame():

        # find and remove leaked actors

        for actor in world_scoped_services.unreal_service.find_actors_by_name(actor_name=_STABLE_NAME_CAMERA_SENSOR, uclass="AActor"):
            world_scoped_services.unreal_service.destroy_actor(actor=actor)

        # initialize segmentation service
        world_scoped_services.segmentation_service.initialize()

        # get viewport info
        viewport_desc = world_scoped_services.rendering_service.get_current_viewport_desc()
        world_desc["viewport_desc"] = viewport_desc

        # compute image dims
        viewport_size_x = viewport_desc["viewport_size_x"]
        viewport_size_y = viewport_desc["viewport_size_y"]
        if viewport_size_x >= viewport_size_y:
            image_width = _MAX_IMAGE_DIM
            image_height = int(_MAX_IMAGE_DIM * viewport_size_y / viewport_size_x)
        else:
            image_height = _MAX_IMAGE_DIM
            image_width = int(_MAX_IMAGE_DIM * viewport_size_x / viewport_size_y)

        # spawn camera sensor

        bp_cam_uclass = world_scoped_services.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        world_desc["camera_sensor"] = world_scoped_services.unreal_service.spawn_actor(uclass=bp_cam_uclass, spawn_parameters={"ObjectFlags": ["RF_Transient"]})
        world_scoped_services.unreal_service.set_stable_name_for_actor(actor=world_desc["camera_sensor"], stable_name=_STABLE_NAME_CAMERA_SENSOR)

        # get camera components
        world_desc["camera_components"] = {
            "final_tone_curve_hdr": world_scoped_services.unreal_service.get_component_by_name(actor=world_desc["camera_sensor"], component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D"),
            "object_ids_uint8": world_scoped_services.unreal_service.get_component_by_name(actor=world_desc["camera_sensor"], component_name="DefaultSceneRoot.sp_object_ids_uint8_", uclass="USpSceneCaptureComponent2D"),
            "sp_depth_meters": world_scoped_services.unreal_service.get_component_by_name(actor=world_desc["camera_sensor"], component_name="DefaultSceneRoot.sp_depth_meters_", uclass="USpSceneCaptureComponent2D"),
            "sp_world_position": world_scoped_services.unreal_service.get_component_by_name(actor=world_desc["camera_sensor"], component_name="DefaultSceneRoot.sp_world_position_", uclass="USpSceneCaptureComponent2D"),
            "world_normal": world_scoped_services.unreal_service.get_component_by_name(actor=world_desc["camera_sensor"], component_name="DefaultSceneRoot.world_normal_", uclass="USpSceneCaptureComponent2D")}

        # align camera with viewport and configure components
        world_scoped_services.rendering_service.align_camera_with_viewport(
            camera_sensor=world_desc["camera_sensor"],
            camera_components=list(world_desc["camera_components"].values()),
            viewport_desc=viewport_desc,
            widths=image_width,
            heights=image_height)

        # initialize camera components
        for component in world_desc["camera_components"].values():
            component.Initialize()
            component.initialize_sp_funcs()

    with _instance.end_frame():
        pass

    _instance.step(num_frames=2)
    _log(f"Actors initialized: {image_width}x{image_height} camera, {'perspective' if viewport_desc['is_perspective'] else 'orthographic'} projection.")

    return True, world_desc


def _terminate_actors(world_desc):

    world_scoped_services = world_desc["world_scoped_services"]

    with _instance.begin_frame():
        for component in world_desc["camera_components"].values():
            component.terminate_sp_funcs()
            component.Terminate()

        if world_desc["camera_sensor"] is not None:
            world_scoped_services.unreal_service.destroy_actor(actor=world_desc["camera_sensor"])

        world_scoped_services.segmentation_service.terminate()

    with _instance.end_frame():
        pass

    _log("Actors terminated.")


def _get_image_data(world_desc, key):

    # let temporal anti-aliasing etc accumulate additional information across multiple frames
    _instance.step(num_frames=2)

    camera_components = world_desc["camera_components"]
    viewport_desc = world_desc["viewport_desc"]
    world_scoped_services = world_desc["world_scoped_services"]

    camera_location = viewport_desc["camera_location"]
    camera_rotation = viewport_desc["camera_rotation"]

    with _instance.begin_frame():
        pass
    with _instance.end_frame():
        raw_final_tone_curve_hdr = camera_components["final_tone_curve_hdr"].read_pixels()["arrays"]["data"].copy()
        raw_object_ids_uint8 = camera_components["object_ids_uint8"].read_pixels()["arrays"]["data"].copy()
        raw_sp_depth_meters = camera_components["sp_depth_meters"].read_pixels()["arrays"]["data"].copy()
        raw_sp_world_position = camera_components["sp_world_position"].read_pixels()["arrays"]["data"].copy()
        raw_world_normal = camera_components["world_normal"].read_pixels()["arrays"]["data"].copy()

        # get segmentation data via segmentation service
        segmentation_id_image, segmentation_id_descs = world_scoped_services.segmentation_service.get_segmentation_data(object_ids_bgra_uint8_image=raw_object_ids_uint8)

    # convert raw buffers to preferred formats

    final_tone_curve_hdr = raw_final_tone_curve_hdr[:,:,[2,1,0]] # BGRA to RGB
    depth_meters = raw_sp_depth_meters[:,:,0].astype(np.float32)
    world_position = raw_sp_world_position[:,:,[0,1,2]].astype(np.float32)
    world_normal = raw_world_normal[:,:,[0,1,2]].astype(np.float32)

    # compute camera-space derived buffers

    R_world_from_unreal_camera = spear.math.to_numpy_matrix_from_spear_rotator(spear_rotator=camera_rotation, as_matrix=True)
    R_unreal_camera_from_world = R_world_from_unreal_camera.T
    M_hypersim_camera_from_world = _M_HYPERSIM_CAMERA_FROM_UNREAL_CAMERA*R_unreal_camera_from_world

    h, w = world_normal.shape[:2]

    M_world_normals = np.matrix(world_normal[:, :, :3].reshape(-1, 3)).T
    M_hypersim_camera_normals = M_hypersim_camera_from_world*M_world_normals
    camera_normal = M_hypersim_camera_normals.T.A.reshape(h, w, 3).astype(np.float32)

    t_camera_location = spear.math.to_numpy_array_from_spear_vector(spear_vector=camera_location, as_matrix=True)
    M_world_positions = np.matrix(world_position[:, :, :3].reshape(-1, 3)).T
    M_hypersim_camera_positions = M_hypersim_camera_from_world*(M_world_positions - t_camera_location)
    camera_position = M_hypersim_camera_positions.T.A.reshape(h, w, 3).astype(np.float32)

    # store image data in world_desc[key]

    world_desc[key] = {
        "final_tone_curve_hdr": final_tone_curve_hdr,
        "depth_meters": depth_meters,
        "world_position": world_position,
        "world_normal": world_normal,
        "camera_normal": camera_normal,
        "camera_position": camera_position,
        "segmentation_id_image": segmentation_id_image,
        "segmentation_id_descs": segmentation_id_descs}

    _log(f"Image data captured ({key}): {len(segmentation_id_descs)} segmentation descs.")


def _save_images(world_desc, key):

    save_dir = os.path.join(_SAVE_DIR, key)
    os.makedirs(save_dir, exist_ok=True)

    data = world_desc[key]
    final_tone_curve_hdr = data["final_tone_curve_hdr"]
    depth_meters = data["depth_meters"]
    camera_normal = data["camera_normal"]
    camera_position = data["camera_position"]
    segmentation_id_image = data["segmentation_id_image"]
    segmentation_id_descs = data["segmentation_id_descs"]

    # final_tone_curve_hdr
    plt.imsave(os.path.join(save_dir, "final_tone_curve_hdr.png"), final_tone_curve_hdr)

    # depth_meters: clamp to [vmin, min(vmax, vmin + 7.5)] meters then scale to [0, 1]
    finite = depth_meters[np.isfinite(depth_meters)]
    vmin = finite.min()
    vmax = min(finite.max(), vmin + 7.5)
    plt.imsave(os.path.join(save_dir, "depth_meters.png"), np.clip((depth_meters - vmin) / (vmax - vmin), 0.0, 1.0))

    # camera_normal
    plt.imsave(os.path.join(save_dir, "camera_normal.png"), np.clip((1.0 + camera_normal[:, :, :3]) / 2.0, 0.0, 1.0))

    # camera_position: per-channel median -> 0.5 (middle grey), uniform K = min(2 * max_channel_abs_dev, 750)
    finite_all_mask = np.all(np.isfinite(camera_position), axis=2)
    finite_pixels = camera_position[finite_all_mask]
    median = np.median(finite_pixels, axis=0)
    K = min(float(2.0 * np.abs(finite_pixels - median).max(axis=0).max()), 750.0)
    plt.imsave(os.path.join(save_dir, "camera_position.png"), np.clip((camera_position - median) / K + 0.5, 0.0, 1.0))

    # segmentation: random HSV colors per ID, index 0 = black
    segmentation_colors = np.zeros((len(segmentation_id_descs), 3))
    for i in range(1, len(segmentation_id_descs)):
        segmentation_colors[i] = np.array(colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0))
    plt.imsave(os.path.join(save_dir, "segmentation_colors.png"), segmentation_colors[segmentation_id_image])


def _update_exec_namespace(world_desc, key):
    global _before_execute_populated, _after_execute_populated

    _exec_namespace["viewport_desc"] = world_desc["viewport_desc"]
    _exec_namespace[key] = world_desc[key]

    if key == "before_execute":
        _before_execute_populated = True
    elif key == "after_execute":
        _after_execute_populated = True
    else:
        assert False


def _validate_code(code):

    try:
        tree = ast.parse(code)
    except SyntaxError as e:
        _log(f"ERROR: SyntaxError: {e}")
        return False

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            _log(f"ERROR: Import statement not allowed: import {', '.join(alias.name for alias in node.names)}")
            return False
        if isinstance(node, ast.ImportFrom):
            _log(f"ERROR: Import statement not allowed: from {node.module} import ...")
            return False
        if isinstance(node, ast.Name) and node.id in _BANNED_NAMES:
            _log(f"ERROR: Name not allowed: {node.id}")
            return False
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Name) and func.id in _BANNED_CALLS:
                _log(f"ERROR: Call not allowed: {func.id}()")
                return False
            if isinstance(func, ast.Attribute) and func.attr in _BANNED_CALLS:
                _log(f"ERROR: Call not allowed: .{func.attr}()")
                return False
        if isinstance(node, ast.Attribute) and node.attr in _BANNED_ATTRS:
            _log(f"ERROR: Attribute access not allowed: .{node.attr}")
            return False

    return True


def _log_status():

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

    # log available variables
    _log("Available variables:")
    for name, desc in _EXEC_NAMESPACE_DESCS.items():
        if name not in _exec_namespace:
            continue
        stale_suffix = ""
        if name == "before_execute" and not _before_execute_populated:
            stale_suffix = " (STALE — not populated this call)"
        elif name == "after_execute" and not _after_execute_populated:
            stale_suffix = " (STALE — not populated this call)"
        _log(f"  {name}: {desc}{stale_suffix}")

    # log inner keys of before_execute / after_execute dicts
    _log("Inner keys of before_execute / after_execute dicts:")
    for name, desc in _RENDER_DATA_DESCS.items():
        _log(f"  {name}: {desc}")

    # log saved images
    _log(f"Saved images in {_SAVE_DIR}:")
    for key, populated in [("before_execute", _before_execute_populated), ("after_execute", _after_execute_populated)]:
        stale_suffix = "" if populated else " (STALE — not saved this call)"
        _log(f"  {key}/{stale_suffix}")
        for filename, desc in _SAVED_IMAGE_DESCS.items():
            _log(f"    {filename}: {desc}")


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
# Main
#

if __name__ == "__main__":
    _log("Server starting...")
    _mcp.run(transport="stdio")
