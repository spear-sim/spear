# Style Guide Sync

`CLAUDE.md` and `.cursor/rules/local-style.mdc` are the two style guides for this repo and must always be kept in sync. Any change to one must be reflected in the other.

`.claudeignore` and `.cursorignore` are the two AI tool ignore files for this repo and must always be kept in sync with each other. Any change to one must be reflected in the other. These are separate from `.gitignore` and should not be synchronized with it.

# SPEAR MCP Server

The MCP tools must be called in this order:
1. `get_status` — call at the start of a session. Reports which scopes (game, editor) are available and whether the camera/proxy manager are initialized. Auto-initializes on first call. No need to call again unless you want to check status explicitly — every `execute_*` call returns status automatically.
2. `execute_code` / `execute_editor_code` / `execute_editor_code_across_frames` — as many times as needed. All cameras automatically sync to their respective viewports before each call. Each call auto-initializes if needed (e.g., after PIE starts/stops) and returns current status at the end of every response.
   These tools run code in two separate Python environments with independent namespaces:
   - **MCP server process** (`execute_code`): Has the SPEAR API (`instance`, `game`, `editor`, `camera_components`, `spear`, `np`, `math`). You manage `begin_frame`/`end_frame` yourself. Variables persist across calls.
   - **Unreal Editor embedded Python** (`execute_editor_code`, `execute_editor_code_across_frames`): Has `math`, `np`, `spear`, `unreal`, and `spear.editor` helpers. `execute_editor_code` is automatically wrapped in a single frame. `execute_editor_code_across_frames` spans multiple editor frames — code must use the `@spear.editor.script` decorator with `yield` to advance the frame (see `examples/control_editor` and `examples/editor_script_interop`). Variables persist across calls.
   Which tool to use:
   - Game world interaction (actors, physics, gameplay): `execute_code` (only option).
   - Rendering images: `execute_code` with `save_images` parameter.
   - Editor world interaction: either `execute_code` (via `editor` scoped services) or `execute_editor_code` (via `unreal` module) — use whichever is more natural for the task.
   - Multi-frame editor operations (screenshots, async waits): `execute_editor_code_across_frames`.

If you encounter an error, the next `execute_*` call will auto-reinitialize if the scope changed.

Before writing `execute_code` snippets, read `docs/running_our_example_applications.md` for a brief description of each example, then read the most relevant example's `run.py`. If you need more detail about a specific example, check its `README.md`. When hitting an unfamiliar API, search `examples/` for a matching example rather than guessing at function signatures.

Usage notes for `execute_code`:
- `spear`, `np`, `math` are always available. Do not use `import` statements.
- `instance` is always available. `game` is available if game world is initialized (PIE running). `editor` is available if running with editor. Check the status returned by `get_status` or at the end of any `execute_*` response to know what's available. `camera_components` is a dict with keys `"final_tone_curve_hdr"` (RGB, uint8 BGRA), `"object_ids_uint8"` (segmentation, uint8 BGRA), `"sp_depth_meters"` (depth in meters, float16, single channel), `"sp_world_position"` (XYZ world coordinates in Unreal units, float16, 3 channels), and `"world_normal"` (surface normals in world space, float16, 3 channels). Use `camera_components["final_tone_curve_hdr"].read_pixels()` inside `end_frame` to get rendered data. `proxy_component_manager` is available for mapping segmentation IDs to actors (call `proxy_component_manager.GetComponentAndMaterialDescs()` each time, since actors may have changed).
- `execute_code` does not wrap in frame blocks — you must use `with instance.begin_frame():` / `with instance.end_frame(): pass` yourself.
- Use `spear.log(...)` for output.
- Use the `save_images` parameter on `execute_code` to save numpy arrays as PNG files for visual inspection. Pass a list of variable names from the exec namespace (e.g., `save_images=["final_tone_curve_hdr", "object_ids_uint8"]`). Images are saved to `tools/tmp/spear-mcp/` and can be viewed with the Read tool. `save_images` uses OpenCV internally, so pass arrays in BGR/BGRA channel order (i.e., the raw output from `read_pixels()` without channel reordering). Use variable names that match the `camera_components` dictionary keys so file names are self-documenting. Only render and save the modalities needed for the current task — e.g., RGB alone for visual verification, RGB + segmentation for object identification, world-position for spatial reasoning.

API patterns for `execute_code`:
- UFUNCTIONs return `UnrealObject` by default — use them directly, pass them to other functions, chain calls (e.g., `comp.GetOwner().K2_GetActorLocation()`). This is the primary way to work with the SPEAR API. Avoid extracting handles unless you have a specific reason.
- `as_dict=True` is only needed when you need to access out parameters that aren't the formal return value (e.g., `GetActorBounds` where `Origin` and `BoxExtent` are out params). For functions with only a return value, omit `as_dict` and use the result directly.
- `as_handle=True` is rarely needed — only for special cases like stuffing handles into numpy arrays for batch operations.
- `spear.to_handle()` is only needed for converting hex pointer strings (from `FComponentAndMaterialDesc`) to integer handles. `spear.to_ptr(handle=...)` is only needed when passing a raw integer handle as a UFUNCTION argument that expects a UObject pointer; prefer passing an `UnrealObject` instead.
- `read_pixels()` returns a dict, not a raw array. Extract the numpy array via `result["arrays"]["data"]`.
- All UFUNCTION calls (e.g., `GetOwner()`, `K2_SetActorLocation`, `GetComponentAndMaterialDescs`) must be inside `begin_frame`/`end_frame` blocks. Calling them outside will assert.
- `get_unreal_object(uobject=handle, with_sp_funcs=False)` lives on scoped services (`game` / `editor`), not on `unreal_service`.
- To go from a segmentation ID to an actor: call `proxy_component_manager.GetComponentAndMaterialDescs()` to get component descs, then for each desc convert the hex string via `game.get_unreal_object(uobject=spear.to_handle(obj=desc["component"]), with_sp_funcs=False)` to get the component, then `.GetOwner()` to get the actor. Use `game.unreal_service.try_get_stable_name_for_actor(actor=...)` for the stable name. In editor Python (`execute_editor_code`), use `proxy_mgr.get_component_and_material_descs(include_debug_info=True)` and resolve handles via `unreal.SpFuncUtils.to_object_from_handle(handle=int(desc.get_editor_property("component"), 16))`.
- `GetComponentAndMaterialDescs` returns a list of descs. Each entry has `componentAndMaterialId`, `component`, `material`, `componentName`, `materialName`, etc. `component` and `material` are hex pointer strings — pass them through `spear.to_handle()` to get integer handles. Pass `bIncludeDebugInfo=True` (or `include_debug_info=True` in editor Python) for convenience name strings; the handles alone are sufficient to resolve everything. Each (component, material) pair has a unique `componentAndMaterialId`. Multiple IDs can refer to the same component (since a component can have multiple materials), and multiple IDs can refer to the same material (since a material can be assigned to multiple components). Always resolve the component pointer from the specific desc matching your target segmentation ID.
- `K2_SetActorLocation` / `K2_SetActorRotation` return False if the actor has static mobility. Call `actor.K2_GetRootComponent().SetMobility(NewMobility="Movable")` first.
- `K2_SetActorLocation` / `K2_SetActorRotation` also fail silently in editor-only mode (no PIE), even with Movable mobility. Use `K2_SetWorldLocation` / `K2_SetWorldRotation` on the root component instead (get it via `scoped_services.unreal_service.get_component_by_name(actor=..., component_name="DefaultSceneRoot", uclass="USceneComponent")`).
- Prefer `K2_SetActorLocationAndRotation` over `K2_SetActorRotation` — the latter can crash and corrupt frame state.
- When changing mobility and then moving/rotating, split across separate `execute_code` calls. Doing both in the same frame block can crash.
- `GetActorBounds(bOnlyCollidingComponents=False, as_dict=True)` returns `{"Origin": {"x": ..., ...}, "BoxExtent": {"x": ..., ...}}`. `BoxExtent` values are **half-extents** (half the full width/height/depth). `as_dict=True` is needed here because `Origin` and `BoxExtent` are out parameters.

Spawning light actors:
- Spawn with `game.unreal_service.spawn_actor(uclass="ASpotLight", location=...)` (or `"APointLight"`, `"ARectLight"`).
- Set mobility to Movable on the root component before moving: `light.K2_GetRootComponent().SetMobility(NewMobility="Movable")`. Split mobility and move into separate `execute_code` calls.
- Position and aim with `K2_SetActorLocationAndRotation`. Do not use `K2_SetActorRotation` (crashes). To aim at a target, use `unreal.MathLibrary.find_look_at_rotation(start, target)` in editor Python rather than manually computing Euler angles. See `python/spear/utils/pipeline_utils.py` for a detailed discussion of Unreal's Euler angle conventions (axis definitions, sign negation, application order).
- Get the light component via `game.unreal_service.get_component_by_class(actor=light, uclass="USpotLightComponent")` (or `"UPointLightComponent"`, etc.).
- Set properties via UFUNCTIONs: `SetIntensity(NewIntensity=...)`, `SetAttenuationRadius(NewRadius=...)`, `SetOuterConeAngle(NewOuterConeAngle=...)`, `SetInnerConeAngle(NewInnerConeAngle=...)`.
- In editor mode (no PIE), spawn via `execute_editor_code` using `unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.SpotLight, location)` and set properties via `set_editor_property`.

Spawning duplicate static mesh actors:
- To duplicate an existing static mesh actor, get its mesh via `comp.StaticMesh.get()` and its scale via `actor.GetActorScale3D()`.
- Spawn with `game.unreal_service.spawn_actor(uclass="AStaticMeshActor", location=...)`.
- On the new actor's `StaticMeshComponent`, call `SetMobility(NewMobility="Movable")` **before** `SetStaticMesh(NewMesh=...)` — without this, the mesh will not render (bounds stay zero).
- Copy the original actor's scale with `SetActorScale3D(NewScale3D=...)`.
- Call `instance.flush(num_frames=3)` after setup for the mesh to appear.
- Materials come from the mesh asset itself (no need to copy override materials unless the original uses them).

Rendering images for visual inspection:
- `get_status` automatically sets up an RGB and segmentation camera that tracks the active viewport. Use `read_pixels()` directly — no manual camera setup needed.
- Do not assume the camera is in the same position as it was when responding to a previous prompt. The camera auto-syncs to the viewport before each `execute_code` call, so its pose is always up-to-date — query it if you need to verify position before taking action (no rendering needed).
- See `examples/render_image/run.py` for the camera sensor pattern and `examples/render_image_hypersim/run.py` for segmentation/world-position images and matching them to the actor list.
- You can spawn additional independent cameras and position them freely to inspect the scene from different angles. For these, use small image sizes (e.g., 256x256 or 512x512).
- When a query references a visible object (e.g., "move the chair I'm looking at"), render an RGB image, visually inspect it to identify the object, then cross-reference the corresponding pixels in the segmentation image to get the actor name.
- Always interpret spatial references ("the wall with paintings", "those chairs", "on the right") relative to what is currently visible in the viewport, not based on absolute world coordinates or full scene enumeration. Render the current viewport first, visually identify what the user is referring to, then act.
- **Never resolve user references by scanning actor/material names from the full scene list.** Always render RGB + segmentation, scan the segmentation image for the IDs that are actually visible in the viewport, and only resolve those IDs to actors. If the user says "those paintings", the correct paintings are the ones visible in the viewport — not whichever entries in `GetComponentAndMaterialDescs` have "Picture" in the name.

Rendering lifecycle for additional cameras — split across separate `execute_code` calls:
1. **Setup** — spawn camera(s), configure and `Initialize()` / `initialize_sp_funcs()` components. All variables persist.
2. **Render** (repeatable) — move camera, `instance.flush()`, `read_pixels()`. Repeat as many times as needed.
3. **Cleanup** — for each component: `terminate_sp_funcs()`, then `Terminate()`. Then `destroy_actor()` for the camera. All inside a `begin_frame`/`end_frame` block.

# Local Style First

- Treat nearby code as the canonical style; match naming, spacing, casing, and call patterns in the file being edited.
- Prefer minimal diffs: change only what is needed for the requested behavior. Check `git diff` against the base branch to verify minimality before presenting changes.
- Prefer simple, direct code over extra helpers or temporary variables unless they improve clarity.
- Keep logging/noise low; do not add extra debug output unless requested.
- Prefer single-line calls when they comfortably fit and remain readable. When a call with keyword arguments doesn't fit on one line, put one argument per line.
- Omit explicit default arguments when behavior is unchanged.
- Prefer division by `2.0` over multiplication by `0.5`.
- When working with transforms in this repo, prefer `to_numpy_array_from_vector(..., as_matrix=True)` and matrix multiplication style.
- For transform setup variables, use ordering `location`, then `rotation`, then `scale` when practical.
- For per-axis scaling in matrix code paths, prefer a diagonal scale matrix (e.g., `np.diag(...)`) and matrix multiplication.
- Avoid redundant conversion chains (e.g., `as_matrix=True` followed immediately by `.A1`).
- Prefer direct `*` operations over `np.multiply(...)` for NumPy arrays.
- When using `np.matrix`, prefer `*` for matrix multiplication instead of `@`.
- When implementing UFUNCTION/UPROPERTY declarations in C++, use Unreal naming conventions for parameter names (e.g., `DeltaTime`, `NewLocation`). Local variables inside function bodies use snake_case (e.g., `level_editor_viewport_client`). When overriding non-UFUNCTION virtual methods, preserve the method name capitalization from the base class but replace parameter names with our naming conventions (e.g., `graph_builder` not `GraphBuilder`).
- When calling Unreal UFUNCTIONs, use keyword argument names with exact Unreal casing.
- When calling `spear.*` functions, always use keyword arguments, even for single-argument calls (except variadic helpers like `entry_point_caller.call_on_game_thread`).
- `as_dict=True` returns a dict whose top-level keys exactly match the UFUNCTION parameter names (plus `"ReturnValue"` for non-void returns). Sub-dictionaries within those values use startingLowerCamelCase keys.
- In SPEAR Python code, specify Unreal enums as unqualified strings (e.g., `"DoubleBuffered"` not `"ESpBufferingMode::DoubleBuffered"`).
- Do not use type annotations in Python code (no `def f(x: int) -> str:`, no variable annotations).
- In argparse, do not pass `description=` to `ArgumentParser()` or `help=` to `add_argument()`. Use `assert args.x in [...]` after `parse_args()` instead of `choices=[...]`.
- Prefer `if-return-else-return` over `if-return-return` (always use explicit `else` for the alternate branch).
- Do not include message text in `assert` statements (use bare `assert condition`, not `assert condition, "message"`).
- In Python type-dispatch if-else chains, place the most general/catch-all branches last.
- In `__init__.py` files, keep imports alphabetical per line, grouped by category.
- Use `os.path.realpath(os.path.join(...))` for constructing file paths.
- When running a subprocess, always log the command before executing it.
- When writing a file (e.g., an image), log the output path.
- Practice strict IWYU (Include What You Use) in C++ files. Every file includes exactly what it needs directly — no relying on transitive includes, not even between a .cpp file and its own header.
- In C++ files, organize includes in groups separated by blank lines, in this order: (1) self-header (.cpp only, first), (2) C standard library (`<stdint.h>`), (3) C++ standard library (`<array>`, `<chrono>`, `<memory>`), (4) Unreal Engine headers (`<Components/...>`, `<RHI...>`), (5) SPEAR headers (`"SpCore/..."`, `"SpServices/..."`), (6) local module headers (`"SpUnrealTypes/..."`), (7) generated header (`"*.generated.h"`, header files only, very last). Sort alphabetically by full include path (case-insensitive) within each group. Inline comments on includes listing non-obvious symbol names should be sorted alphabetically, and aligned within each group so that `//` starts exactly one space after the end of the longest `#include` line **that has a comment** in the group (not the longest include overall). Whether to add a comment depends solely on how non-obvious the symbol is (e.g., `<utility>` needs `// std::move` because that mapping is non-obvious; `<string>` does not need `// std::string`). If a new commented include becomes the new longest, update all other commented lines in the group to match the new column.
- In C++ headers, preserve and update comments documenting thread-access patterns on member variables. Inline comments on groups of related member variables should be aligned to the same column (one space after the longest declaration **that has a comment** in the group), using the same alignment principle as include comments.
- In C++ code, use same-line (Egyptian) braces for control flow statements (`if (...) {`, `for (...) {`, `else {`).
- Avoid ternary operators (`?:`) in C++ code unless there is a very good reason; prefer explicit `if`/`else`.
- For C++ container teardown, use `clear()` alone; do not call `shrink_to_fit()` (it is only a non-binding request).
- Prefer aggregate assignment for `std::array` initialization and teardown (e.g., `readback_buffers_ = { make_unique(...), nullptr }`) over per-element `.at(i) = ...` or `.at(i).reset()`.
- Suffix C++ functions that execute on the render thread with `_RenderThread` (e.g., `copyPixelsFromStagingToCPU_RenderThread`).
- For C++ `enum class` declarations, assign explicit integer values with aligned `=` signs (e.g., `Idle = 0`, `Pending = 1`). See `EngineService.h` for reference.
- When adding an `_in_editor_script` generator variant of an existing method, use `yield` only as pause points (after each `begin_frame`/`end_frame` context block) and `return` for the value; use `yield from` to delegate to sub-generators. Factor shared setup into a single `_impl` helper that returns resolved parameters rather than duplicating the impl for regular and generator variants.
- When adding a new buffering/readback mode, keep the same SpFunc interface (`enqueue_copy` + `read_pixels`) and express mode differences through priming frame count and internal dispatch. In C++ if-else chains checking `BufferingMode`, order branches as SingleBuffered, DoubleBuffered, TripleBuffered.
- Prefer generic byte buffers (`std::vector<uint8_t>` with `SpAlignedAllocator`) and `SpArrayDataTypeUtils::getSizeOf` over typed containers with per-type if-else chains for pixel data.
- When a synchronous code path (e.g., single-buffered readback with `FlushRenderingCommands`) can write directly to the final output buffer, avoid intermediate copies through a scratchpad.
- For render target texture formats in asset creation scripts, match `num_channels_per_pixel` and texture format to the actual data dimensionality (e.g., `RTF_R16F`/`RTF_R8` for scalar modalities, not `RTF_RGBA16F`/`RTF_RGBA8`).
- In Python visualization lambdas for image data, explicitly index channels (e.g., `data[:,:,[0,1,2]]` for RGB, `data[:,:,[0,0,0]]` for greyscale) rather than passing arrays that may include alpha.
- Keep lists of component descriptors sorted alphabetically by name.
- Never remove existing diagnostic logging (spin-wait counts, warnings, etc.) without being explicitly asked. Diagnostic messages are critical for understanding runtime behavior.
- If unsure between two styles, follow the dominant pattern already present in the touched file.
- When in doubt how to interpret any convention listed here, look at the existing source code, which should be 99% consistent already and is the canonical reference.
