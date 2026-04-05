# SPEAR MCP Server

## Tools and execution environments

The MCP tools must be called in this order:
1. `initialize` — call at the start of a session. Reports which worlds (game, editor) are available. No need to call again unless you want to check status explicitly — every `execute_*` call returns status automatically.
2. `execute_code` / `execute_editor_code` / `execute_editor_code_across_frames` — as many times as needed. All cameras automatically sync to their respective viewports before each call. Each call auto-initializes if needed (e.g., after PIE starts/stops) and returns current status at the end of every response.
3. `terminate` — optional. Call at the end of a session to clean up persistent actors (e.g., `ASpStableNameManager`) and restore the scene to its original state. If not called, these actors remain in the scene but are harmless.

These tools run code in two separate Python environments with independent namespaces:
- **MCP server process** (`execute_code`): Pre-imported symbols: `instance`, `game`, `editor`, `camera_components`, `proxy_component_manager`, `spear`, `np`, `math`. You manage `begin_frame`/`end_frame` yourself. Variables persist across calls.
- **Unreal Editor embedded Python** (`execute_editor_code`, `execute_editor_code_across_frames`): Pre-imported symbols: `math`, `np`, `spear`, `unreal`, and `spear.editor` helpers. No other imports are available — all `import` statements are banned by the code validator, as are names like `os`, `sys`, `pathlib`, etc. `execute_editor_code` is automatically wrapped in a single frame. `execute_editor_code_across_frames` spans multiple editor frames — code must use the `@spear.editor.script` decorator with `yield` to advance the frame (see `examples/control_editor` and `examples/editor_script_interop`). Variables persist across calls. **Do not create a new `spear.Instance()` inside editor code via MCP** — the MCP server already manages its own instance and creating a second one will time out.

Which tool to use by session type:
- **Standalone game** (no editor): `execute_code` — the only option. `game` is available, `editor` is not.
- **Editor with PIE running**: `execute_code` — gives access to `game`, `editor`, `camera_components`, and `save_images` all in one environment.
- **Editor without PIE**: prefer `execute_code` — it can render images (`camera_components` + `save_images`) and interact with the editor world (`editor` scoped services) in the same call. Use `execute_editor_code` when the `unreal` module API is more natural for the task (e.g., `unreal.EditorLevelLibrary`, `set_editor_property`), but note that rendering and segmentation are not available there.
- **Multi-frame editor operations** (screenshots, async waits): `execute_code` with multiple `begin_frame`/`end_frame` blocks, or `execute_editor_code_across_frames` with the `@spear.editor.script` decorator and `yield` to advance frames.
- **Rendering images**: always `execute_code` with `save_images` parameter (only option).

Do not run `tools/run_mcp_server.py` directly via Bash — it is a stdio MCP server that blocks indefinitely. Only reference it in MCP config.

## Error recovery and state management

- The MCP server is stateless per-call for infrastructure (cameras, proxy manager), but agent-defined variables in the exec namespace survive across `execute_*` calls by default.
- If a C++ exception or assert occurs during execution, the server automatically recovers on the next call by re-initializing the engine service. No manual intervention is needed.
- The server detects world changes (e.g., the user opened a different level) by comparing `world_id` values across calls. If any world changed, all agent-defined variables are automatically cleared. The agent is notified via the log message "World state changed since last call. Clearing agent variables." Standard variables (`instance`, `game`, `editor`, `camera_components`, `proxy_component_manager`, `spear`, `np`, `math`) are always re-populated regardless.

## Logging and debugging

- The MCP server writes all log messages to `tools/tmp/spear-mcp/mcp_server.log`. Previous session logs are rotated to `mcp_server.prev.log` on server startup.
- Every `_log(...)` call in the server writes to both the log file and an in-memory buffer that is returned to the caller via `_get_log()`. This is the only way to see what happened during a tool call — MCP tools are opaque and stderr/tracebacks are not visible unless explicitly captured.
- If something goes wrong and the tool returns an unhelpful error, check the log file for the full traceback.

## Actor naming

- When spawning actors, always call `game.unreal_service.set_stable_name_for_actor(actor=..., stable_name="...")` to assign a human-readable name. This makes the actor discoverable via `find_actors_by_name`, `find_actors_as_dict`, and similar `_as_dict` methods that key by stable name.
- Without a stable name, spawned actors will only be findable by class or tag, not by name.
- The MCP server spawns an `ASpStableNameManager` at initialization so that `set_stable_name_for_actor` works on all actors, including those that don't have a `USpStableNameComponent` built into their blueprint.

## Usage notes for `execute_code`

- `instance` is always available. `game` is available if game world is initialized (PIE running). `editor` is available if running with editor. Check the status returned by `initialize` or at the end of any `execute_*` response to know what's available. Do not attempt to create your own `spear.Instance` or call `instance.get_game()` or `instance.get_editor()`. Use the variables that are already provided to you.
- `camera_components` is a dict with keys `"final_tone_curve_hdr"` (RGB, uint8 BGRA), `"object_ids_uint8"` (segmentation, uint8 BGRA), `"sp_depth_meters"` (depth in meters, float16, single channel), `"sp_world_position"` (XYZ world coordinates in Unreal units, float16, 3 channels), and `"world_normal"` (surface normals in world space, float16, 3 channels). For example, use `bgra_pixels = camera_components["final_tone_curve_hdr"].read_pixels()["arrays"]["data"]` inside `end_frame` to get rendered pixels.
- `proxy_component_manager` is available for mapping segmentation IDs to actors (call `proxy_component_manager.GetComponentAndMaterialDescs()` before each use since actors may have changed).
- `execute_code` does not wrap in frame blocks — you must use `with instance.begin_frame():` / `with instance.end_frame(): pass` yourself.
- Use `spear.log(...)` for output.
- Use the `save_images` parameter on `execute_code` to save numpy arrays as PNG files for visual inspection. Pass a list of variable names from the exec namespace (e.g., `save_images=["final_tone_curve_hdr", "object_ids_uint8"]`). Images are saved to `tools/tmp/spear-mcp/` and can be viewed with the Read tool. `save_images` uses OpenCV internally, so pass arrays in BGR/BGRA channel order. The `final_tone_curve_hdr` and `object_ids_uint8` components already return BGRA from `read_pixels()`, so their output can be saved directly without channel reordering. Use variable names that match the `camera_components` dictionary keys so file names are self-documenting. Only render and save the modalities needed for the current task — e.g., RGB alone for visual verification, RGB + segmentation for object identification, world-position for spatial reasoning.

Before writing `execute_code` snippets, read `docs/running_our_example_applications.md` for a brief description of each example, then read the most relevant example's `run.py`. If you need more detail about a specific example, check its `README.md`. When hitting an unfamiliar API, search `examples/` for a matching example rather than guessing at function signatures.

# SPEAR API Patterns

## Calling conventions

- UFUNCTIONs return `UnrealObject` by default — use them directly, pass them to other functions, chain calls (e.g., `comp.GetOwner().K2_GetActorLocation()`). This is the primary way to work with the SPEAR API. Avoid extracting handles unless you have a specific reason.
- `as_dict=True` is only needed when you need to access out parameters that aren't the formal return value (e.g., `GetActorBounds` where `Origin` and `BoxExtent` are out params). For functions with only a return value, omit `as_dict` and use the result directly. If you do pass `as_dict=True` on a non-void function, the return value is wrapped under a `"ReturnValue"` key (e.g., `K2_GetActorLocation(as_dict=True)` returns `{"ReturnValue": {"x": ..., "y": ..., "z": ...}}`).
- `as_handle=True` is rarely needed — only for special cases like stuffing handles into numpy arrays for batch operations.
- `spear.to_handle()` is only needed for converting hex pointer strings (from `GetComponentAndMaterialDescs()`) to integer handles. `spear.to_ptr(handle=...)` is only needed when passing a raw integer handle as a UFUNCTION argument that expects a UObject pointer; prefer passing an `UnrealObject` instead.
- `read_pixels()` returns a dict, not a raw array. Extract the numpy array via `result["arrays"]["data"]`. **The returned array is volatile** — its backing memory may be freed or overwritten after the frame ends. Always deep copy with `.copy()` before storing: `data = result["arrays"]["data"].copy()`. Failure to copy can cause segfaults.
- All UFUNCTION calls (e.g., `GetOwner()`, `K2_SetActorLocation`, `GetComponentAndMaterialDescs`) must be inside `begin_frame`/`end_frame` blocks. Calling them outside will assert.
- `get_unreal_object(uobject=handle, with_sp_funcs=False)` lives on scoped services (`game` / `editor`), not on `unreal_service`.
- `component.call_async.<FunctionName>(...)` returns a Future. Retrieve result with `future.get()` in `end_frame`.

## Segmentation ID to actor mapping

- To go from a segmentation ID to an actor: call `proxy_component_manager.GetComponentAndMaterialDescs(as_dict=True)["ReturnValue"]` to get component descs, then for each desc convert the hex string via `editor.get_unreal_object(uobject=spear.to_handle(obj=desc["component"]), with_sp_funcs=False)` to get the component, then `.GetOwner()` to get the actor. Use `editor.unreal_service.try_get_stable_name_for_actor(actor=...)` for the stable name.
- `GetComponentAndMaterialDescs` returns a list of descs. Each entry has `componentAndMaterialId`, `component`, and `material`. `component` and `material` are hex pointer strings — pass them through `spear.to_handle()` to get integer handles. Each (component, material) pair has a unique `componentAndMaterialId`. Multiple IDs can refer to the same component (since a component can have multiple materials), and multiple IDs can refer to the same material (since a material can be assigned to multiple components). Always resolve the component pointer from the specific desc matching your target segmentation ID.

## Moving actors

- `K2_SetActorLocation` / `K2_SetActorRotation` return False if the actor has static mobility. Call `actor.K2_GetRootComponent().SetMobility(NewMobility="Movable")` first.
- `K2_SetActorLocation` / `K2_SetActorRotation` work in editor-only mode (no PIE) thanks to the `GAllowActorScriptExecutionInEditor` guard in `UnrealUtils.cpp`.
- Prefer `K2_SetActorLocationAndRotation` over `K2_SetActorRotation` — the latter can crash and corrupt frame state.
- When changing mobility and then moving/rotating, split across separate `execute_code` calls. Doing both in the same frame block can crash.
- `GetActorBounds(bOnlyCollidingComponents=False, as_dict=True)` returns `{"Origin": {"x": ..., ...}, "BoxExtent": {"x": ..., ...}}`. `BoxExtent` values are **half-extents** (half the full width/height/depth). `as_dict=True` is needed here because `Origin` and `BoxExtent` are out parameters.

## API quick-reference

See `examples/getting_started`, `examples/render_image`, `examples/render_image_hypersim` for full usage.

- `game.unreal_service.load_class(uclass="AActor", name="/Path/To/BP.BP_C")` → UClass object for spawning blueprints.
- `game.unreal_service.spawn_actor(uclass=..., location={"X": float, "Y": float, "Z": float})` → Actor object.
- `game.unreal_service.destroy_actor(actor=...)` → None.
- `game.unreal_service.find_actors_by_class(uclass="APostProcessVolume")` → List of actors.
- `game.unreal_service.get_component_by_name(actor=..., component_name="...", uclass="...")` → Component object. The `component_name` must match the component's path within the actor hierarchy (e.g., `"DefaultSceneRoot.final_tone_curve_hdr_"`), not a generic name.
- `game.unreal_service.get_component_by_class(actor=..., uclass="...")` → Component object.
- `game.unreal_service.get_stable_name_for_actor(actor=...)` / `try_get_stable_name_for_actor(actor=...)` → Stable name string.
- `game.get_unreal_object(uclass="UGameplayStatics")` → Default object for a UClass.
- `game.get_unreal_object(uobject=handle, with_sp_funcs=False)` → Python wrapper from an integer handle.
- `actor.K2_GetActorLocation()` → `{"X": float, "Y": float, "Z": float}`.
- `actor.K2_GetActorRotation()` → `{"pitch": float, "yaw": float, "roll": float}`.
- `actor.GetActorBounds(bOnlyCollidingComponents=False, as_dict=True)` → `{"Origin": {"x":...}, "BoxExtent": {"x":...}}` (half-extents).
- `actor.K2_GetRootComponent()` → Root component object.
- `actor.GetOwner()` → Owning actor object.
- `component.GetNumMaterials()` → int. `component.GetMaterial(ElementIndex=0)` → Material object. `component.SetMaterial(ElementIndex=0, Material=...)` → None.
- `component.read_pixels()` → `{"arrays": {"data": np.ndarray}}`. The array shape is `(height, width, channels)`.

## Rendering images for visual inspection

- Each `execute_*` call automatically sets up cameras that track the active viewport, and tears them down after the call completes. Use `read_pixels()` directly via `camera_components` — no manual camera setup needed.
- Do not assume the camera is in the same position as it was when responding to a previous prompt. The camera auto-syncs to the viewport before each `execute_code` call, so its pose is always up-to-date — query it if you need to verify position before taking action (no rendering needed).
- See `examples/render_image/run.py` for the camera sensor pattern and `examples/render_image_hypersim/run.py` for segmentation/world-position images and matching them to the actor list.
- You can spawn additional independent cameras and position them freely to inspect the scene from different angles. For these, use small image sizes (e.g., 256x256 or 512x512).
- When a query references a visible object (e.g., "move the chair I'm looking at"), render an RGB image, visually inspect it to identify the object, then cross-reference the corresponding pixels in the segmentation image to get the actor name.
- Always interpret spatial references ("the wall with paintings", "those chairs", "on the right") relative to what is currently visible in the viewport, not based on absolute world coordinates or full scene enumeration. Render the current viewport first, visually identify what the user is referring to, then act.
- **Never resolve user references by scanning actor/material names from the full scene list.** Always render RGB + segmentation, scan the segmentation image for the IDs that are actually visible in the viewport, and only resolve those IDs to actors. If the user says "those paintings", the correct paintings are the ones visible in the viewport — not whichever entries in `GetComponentAndMaterialDescs` have "Picture" in the name.

## Additional cameras

Split across separate `execute_code` calls:
1. **Setup** — spawn camera(s), configure and `Initialize()` / `initialize_sp_funcs()` components. All variables persist.
2. **Render** (repeatable) — move camera, `instance.flush()`, `read_pixels()`. Repeat as many times as needed.
3. **Cleanup** — for each component: `terminate_sp_funcs()`, then `Terminate()`. Then `destroy_actor()` for the camera. All inside a `begin_frame`/`end_frame` block.

## Spawning light actors

- Spawn with `game.unreal_service.spawn_actor(uclass="ASpotLight", location=...)` (or `"APointLight"`, `"ARectLight"`).
- Set mobility to Movable on the root component before moving: `light.K2_GetRootComponent().SetMobility(NewMobility="Movable")`. Split mobility and move into separate `execute_code` calls.
- Position and aim with `K2_SetActorLocationAndRotation`. Do not use `K2_SetActorRotation` (crashes). To aim at a target, use `unreal.MathLibrary.find_look_at_rotation(start, target)` in editor Python rather than manually computing Euler angles. See `python/spear/utils/pipeline_utils.py` for a detailed discussion of Unreal's Euler angle conventions (axis definitions, sign negation, application order).
- Get the light component via `game.unreal_service.get_component_by_class(actor=light, uclass="USpotLightComponent")` (or `"UPointLightComponent"`, etc.).
- Set properties via UFUNCTIONs: `SetIntensity(NewIntensity=...)`, `SetAttenuationRadius(NewRadius=...)`, `SetOuterConeAngle(NewOuterConeAngle=...)`, `SetInnerConeAngle(NewInnerConeAngle=...)`.
- In editor mode (no PIE), spawn via `execute_editor_code` using `unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.SpotLight, location)` and set properties via `set_editor_property`.
- Reasonable intensity values for spotlights: 200–1000 for subtle accent lighting, 1000–5000 for moderate illumination, 5000+ for very bright or outdoor lighting. Start low (~500) and increase — the default Unreal intensity units are candelas and values above 5000 easily blow out indoor scenes.

## Spawning duplicate static mesh actors

- To duplicate an existing static mesh actor, get its mesh via `comp.StaticMesh.get()` and its scale via `actor.GetActorScale3D()`.
- Spawn with `game.unreal_service.spawn_actor(uclass="AStaticMeshActor", location=...)`.
- On the new actor's `StaticMeshComponent`, call `SetMobility(NewMobility="Movable")` **before** `SetStaticMesh(NewMesh=...)` — without this, the mesh will not render (bounds stay zero).
- Copy the original actor's scale with `SetActorScale3D(NewScale3D=...)`.
- Call `instance.flush(num_frames=3)` after setup for the mesh to appear.
- Materials come from the mesh asset itself (no need to copy override materials unless the original uses them).
