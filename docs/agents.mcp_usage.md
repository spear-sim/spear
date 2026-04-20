# SPEAR MCP Server

## Tools

Each tool is self-contained — it initializes the engine connection, renders the current viewport, saves visualization images, populates the exec namespace, and tears down per-call actors automatically.

- `get_viewport_data` — renders the current viewport and populates the exec namespace. Call this first to see what's in the scene. No code execution.
- `execute_code(code)` — same setup as `get_viewport_data`, then executes `code` in the MCP server process. You manage `begin_frame`/`end_frame` yourself. Use `spear.log(...)` for output.
- `execute_editor_code(code)` — same setup, then executes `code` in Unreal Editor's embedded Python (single frame, has access to `unreal` module). Use when the `unreal` module API is more natural (e.g., `unreal.EditorLevelLibrary`, `set_editor_property`).

Do not run `tools/run_mcp_server.py` directly via Bash — it is a stdio MCP server. Only reference it in MCP config.

## Available variables

Each tool call populates the exec namespace with these top-level variables:

| Variable | Description |
|---|---|
| `math`, `np`, `spear` | Standard imports |
| `instance` | `spear.Instance` connected to the running engine |
| `game` | Game world scoped services (`None` if no game) |
| `editor` | Editor world scoped services (`None` if no editor) |
| `viewport_desc` | Dict with camera pose, FOV, viewport size |
| `before_execute` | Dict of viewport data captured **before** agent code runs (see inner keys below, identical to `after_execute` when calling the `get_viewport_data` tool) |
| `after_execute` | Dict of viewport data captured **after** agent code runs (see inner keys below, identical to `before_execute` when calling the `get_viewport_data` tool) |

`before_execute` and `after_execute` are dicts with these inner keys:

| Key | Description |
|---|---|
| `final_tone_curve_hdr` | RGB image, uint8, (H,W,3) |
| `depth_meters` | Depth in meters, float32, (H,W) |
| `world_position` | XYZ world coordinates, float32, (H,W,3) |
| `world_normal` | Surface normals in world space, float32, (H,W,3) |
| `camera_normal` | Surface normals in Hypersim camera space, float32, (H,W,3) |
| `camera_position` | Positions in Hypersim camera space, float32, (H,W,3) |
| `segmentation_id_image` | Per-pixel index into `segmentation_id_descs`, int32, (H,W) |
| `segmentation_id_descs` | List of dicts with actor/component/material handles and names |

The `actor`, `component`, and `material` fields in each `segmentation_id_descs` entry are `uint64` handles, not `UnrealObject`s — see **Handles and UnrealObjects** below for how to convert.

**Do not** create a new `spear.Instance()`, call `instance.get_game()`, or `instance.get_editor()`. All `import` statements are banned, as are names like `os`, `sys`, `pathlib`.

Agent-defined variables persist across calls. If the world changes (e.g., user opens a different level), agent variables are automatically cleared.

## Saved images

Each tool call saves visualization images to `tools/tmp/spear-mcp/before_execute/` and `tools/tmp/spear-mcp/after_execute/`. View them with the Read tool.

| File | Description |
|---|---|
| `final_tone_curve_hdr.png` | RGB image |
| `depth_meters.png` | Depth shifted by min, divided by K = min(span, 7.5) meters |
| `camera_normal.png` | Surface normals in Hypersim camera space, (1+n)/2 |
| `camera_position.png` | Positions in Hypersim camera space, per-channel median centered at 0.5, uniform K = min(2 * max_channel_abs_dev, 750) |
| `segmentation_colors.png` | Random colors per segmentation ID |

## At the start of each turn

- **Always call `get_viewport_data` before acting on a new user prompt.** The user can pan, zoom, or switch scenes between turns. Skipping this means stale assumptions about what's visible — including destructive actions (move, rotate, delete) hitting the wrong actors.
- **Don't reuse `segmentation_id_descs` indices across calls.** Entry 5 in one call is not necessarily entry 5 in the next — the index is scene-/viewport-specific and volatile. Re-identify actors by `actorStableName` / `actorUnrealName`, or by a fresh handle lookup, each turn.

## Spatial reasoning — always ground in the images

Scene metadata from Unreal is unreliable for spatial/visual tasks. Treat the rendered images as the source of truth:

- `final_tone_curve_hdr.png` — what the user sees
- `depth_meters` — per-pixel distance from camera
- `world_position` / `camera_position` — exact 3D coordinates at each pixel
- `segmentation_id_image` / `segmentation_id_descs` — per-pixel actor/component/material identity
- `segmentation_colors.png` — useful when object boundaries in the RGB are ambiguous

- **Don't trust actor forward vectors or rotation values to tell you which way something "visually" faces.** Meshes can be authored with any local orientation — the reported forward has no guaranteed relationship to a visual front (seat front, door opening, camera lens direction, etc.). If facing matters, render and look.
- **To recover the mesh-local forward offset, use guess-and-check.** Set the actor's yaw to a known value (e.g., 0°), render, and read the world-space visual forward off the image. The measured offset α lets you convert any desired world direction into an actor yaw. Don't try to infer α from the natural pose via surface-normal averaging (it only measures direction-to-camera) or backrest-height heuristics (biased by occlusion and gives inconsistent results across instances of the same mesh).
- **Don't grep actor names to answer "what is the user looking at?"** Names may not match what's visible, and things with matching names may be off-screen or occluded. Start from the pixels the user is asking about and resolve outward.
- **Compute directions and distances from pixel data, not from metadata.** `world_position[y, x]` gives a 3D point you can trust for opaque surfaces. An actor's location and bounds are fine. A yaw or forward vector alone is not.
- **Translucent surfaces (glass, etc.) appear in `segmentation_id_image` but not in `world_position` / `world_normal` / `depth_meters` / `camera_position` / `camera_normal`.** Those buffers capture the opaque geometry behind the translucent surface. A pixel that segments as "glass" has a world_position out in whatever lies beyond (the yard, the sky). Use the actor's bounds/transform, or restrict pixel queries to opaque components (window frame, not glass panes).
- **Verify by rendering.** Apply a change, re-render, and look at the `after_execute` image. Don't declare success from metadata alone.

## Error recovery

- If a C++ exception or assert occurs, the server automatically recovers on the next call.
- If `execute_code` leaves a dangling `begin_frame` without `end_frame`, the server cleans it up automatically.
- The tool response always reports current status, including whether variables and images are stale.

## Logging

- Log file: `tools/tmp/spear-mcp/mcp_server.log` (previous session rotated to `mcp_server.prev.log`).
- If something goes wrong and the tool returns an unhelpful error, check the log file for the full traceback.

# SPEAR API Patterns

## Calling conventions

- UFUNCTIONs return `UnrealObject` by default — use them directly, pass them to other functions, chain calls (e.g., `comp.GetOwner().K2_GetActorLocation()`).
- `as_dict=True` is only needed for UFUNCTIONs with out parameters that aren't the formal return value. Example: `actor.GetActorBounds(bOnlyCollidingComponents=False, as_dict=True)` returns `{"Origin": {"X": ..., "Y": ..., "Z": ...}, "BoxExtent": {"X": ..., "Y": ..., "Z": ...}}` — both are `out` params, so without `as_dict=True` you can't read them. By contrast, `actor.GetActorScale3D()` returns its `FVector` as its formal return value and doesn't need `as_dict`.
- All UFUNCTION calls must be inside `with instance.begin_frame()` / `with instance.end_frame()` blocks. Each `with instance.begin_frame():` block must be paired with a matching `with instance.end_frame():` block. Starting a new `begin_frame` before closing the previous one will raise an assertion.

## Rotations and Euler angles

Unreal represents rotations as `(Pitch, Yaw, Roll)` Euler angles in degrees, applied in the fixed parent frame in the order **roll → pitch → yaw**. Each individual angle (verified in the editor):

- **Roll** — rotation around X, from +Z toward +Y.
- **Pitch** — rotation around Y, from +X toward +Z.
- **Yaw** — rotation around Z, from +X toward +Y.

Practical consequences:

- A component's local forward is +X. `Pitch=-90` aims it straight down (world -Z); `Yaw=90` aims it along world +Y.
- `scipy.spatial.transform.Rotation` uses a convention that disagrees on the sign of pitch and roll, and expects radians rather than degrees — see the header comment in `python/spear/utils/math_utils.py` for conversion details.
- The `rotation=` kwarg on `spawn_actor` is **composed with** the spawned class's default rotation, not substituted for it. If the class default is non-identity (e.g., `ASpotLight` ships with `Pitch=-90`, pointing down), passing `rotation=...` gives you a compounded orientation rather than the one you asked for. To set an absolute rotation, omit `rotation=` from `spawn_actor` and call `K2_SetActorRotation` afterward; verify with `K2_GetActorRotation`.

## Handles and UnrealObjects

Normal usage is `UnrealObject`s all the way down: `unreal_service.*` helpers (`spawn_actor`, `get_component_by_*`, `find_actors_by_*`, `load_class`, etc.) return `UnrealObject`s, UFUNCTIONs return `UnrealObject`s, and UFUNCTION parameters typed as UObject pointers accept `UnrealObject`s directly. You do not need to touch raw handles to call methods, pass arguments, or chain calls.

Raw `uint64` handles surface in one routine place:

- `segmentation_id_descs` — each entry's `actor`, `component`, and `material` fields are raw handles.

To work with those handles:

- **Handle → UnrealObject** (usually needed to call methods on the segmented actor/component/material): `obj = game.get_unreal_object(uobject=handle, with_sp_funcs=False)`. Use `editor.get_unreal_object(...)` for editor-world objects. Must be called inside `with instance.begin_frame()` or `with instance.end_frame()`.
- **UnrealObject → handle** (rarely needed): `obj.uobject` returns the raw `uint64`. The only common reason to reach for this is comparing an `UnrealObject` against a handle from `segmentation_id_descs` (e.g., `m.uobject == desc["material"]`). To pass a wrapped object to a UFUNCTION, pass the `UnrealObject` itself — do not extract the handle.

## Actor naming

- When spawning actors, call `game.unreal_service.set_stable_name_for_actor(actor=..., stable_name="...")` to assign a human-readable name.
- Without a stable name, spawned actors will only be findable by class or tag, not by name.

## Moving actors

- `K2_SetActorLocation` / `K2_SetActorRotation` return False if the actor has static mobility. Call `actor.K2_GetRootComponent().SetMobility(NewMobility="Movable")` first.
- `GetActorBounds(bOnlyCollidingComponents=False, as_dict=True)` returns `{"Origin": {...}, "BoxExtent": {...}}` where `BoxExtent` values are half-extents.

## Spawning light actors

- Spawn with `game.unreal_service.spawn_actor(uclass="ASpotLight", location=...)` (or `"APointLight"`, `"ARectLight"`).
- Set mobility to Movable before moving: `light.K2_GetRootComponent().SetMobility(NewMobility="Movable")`. Split mobility and move into separate calls.
- Position and aim with `K2_SetActorLocationAndRotation`.
- A spotlight's cone shines along its local +X axis, and `ASpotLight` ships with a default rotation of `Pitch=-90`, so a freshly-spawned spotlight already points straight down. To aim differently, call `K2_SetActorRotation` after spawn (see the `spawn_actor` kwarg caveat in **Rotations and Euler angles**).
- Get the light component via `game.unreal_service.get_component_by_class(actor=light, uclass="USpotLightComponent")`.
- Set properties: `SetIntensity(NewIntensity=...)`, `SetAttenuationRadius(NewRadius=...)`, `SetOuterConeAngle(NewOuterConeAngle=...)`.
- Reasonable intensity: 200–1000 subtle, 1000–5000 moderate, 5000+ bright. Start low (~500).

## Spawning duplicate static mesh actors

- Get the original mesh via `comp.StaticMesh.get()` and scale via `actor.GetActorScale3D()`.
- Spawn with `game.unreal_service.spawn_actor(uclass="AStaticMeshActor", location=...)`.
- On the new actor's `StaticMeshComponent`, call `SetMobility(NewMobility="Movable")` **before** `SetStaticMesh(NewMesh=...)`.
- Copy scale with `SetActorScale3D(NewScale3D=...)`.

## API quick-reference

Concrete kwargs and return shapes for common helpers. See `examples/getting_started`, `examples/render_image`, `examples/render_image_hypersim` for full usage.

**Case convention:** FVector/FRotator dicts **returned** by UFUNCTIONs use lowercase keys (`{"x", "y", "z"}`, `{"pitch", "yaw", "roll"}`). Dicts you **pass in** to setters use uppercase (`{"X", "Y", "Z"}`, `{"Pitch", "Yaw", "Roll"}`) to match Unreal's C++ field names.

### `game.unreal_service` / `editor.unreal_service`

- `spawn_actor(uclass="ASpotLight", location={"X": 0.0, "Y": 0.0, "Z": 0.0}, rotation={"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0})` → Actor. `rotation` is optional and defaults to zeros.
- `destroy_actor(actor=my_actor)` → None.
- `find_actors_by_class(uclass="AStaticMeshActor")` → List of actors.
- `find_actors_by_class_as_dict(uclass="AStaticMeshActor", include_unreal_name=False)` → Dict keyed by stable name.
- `find_actors_by_name(actor_name="my_light", uclass="ASpotLight")` → List (single `actor_name` string, not a list). `actor_name` is the stable name (e.g., `"Meshes/05_chair/LivingRoom_Chair_01"`), not the Unreal name (e.g., `"SM_chair_living_2"`).
- `find_actor_by_name(actor_name="my_light", uclass="ASpotLight")` → Single actor (asserts exactly one match). `actor_name` is the stable name, same as above.
- `load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")` → UClass object that can be passed to other functions as a `uclass=` kwarg. Use when the shorthand `uclass="ASpotLight"` form won't resolve (e.g., for Blueprint classes).
- `set_stable_name_for_actor(actor=my_actor, stable_name="my_light")` → None.
- `get_stable_name_for_actor(actor=my_actor, include_unreal_name=False)` → Stable-name string.
- `get_component_by_class(actor=my_actor, uclass="USpotLightComponent")` → Component (asserts exactly one).
- `get_components_by_class(actor=my_actor, uclass="UStaticMeshComponent")` → List of components.

### Handles ↔ UnrealObjects

- `game.get_unreal_object(uobject=handle_uint64, with_sp_funcs=False)` → Python wrapper. `editor.get_unreal_object(...)` for editor-world objects. Must be called inside `with instance.begin_frame()` / `end_frame()`.
- `obj.uobject` → Raw uint64 handle from an UnrealObject.

### Introspection and UProperty access

- `obj.print_debug_info()` → None. Dumps every UFUNCTION and UProperty on the object's class — use this when you don't know what a class exposes instead of guessing signatures.
- `obj.get_properties()` → Nested dict of all UProperty values on `obj`.
- `obj.SomePropertyName.get()` → Read a UProperty directly. Distinct from calling a UFUNCTION. Example: `actor.RootComponent.get()` returns the root component (equivalent to `actor.K2_GetRootComponent()`).

### Actor UFUNCTIONs

- `actor.K2_GetActorLocation()` → `{"x": float, "y": float, "z": float}`.
- `actor.K2_GetActorRotation()` → `{"pitch": float, "yaw": float, "roll": float}`.
- `actor.K2_SetActorLocationAndRotation(NewLocation={"X": 0.0, "Y": 0.0, "Z": 0.0}, NewRotation={"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0}, bSweep=False, bTeleport=True)` → bool.
- `actor.K2_SetActorRotation(NewRotation={"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0}, bTeleportPhysics=True)` → bool.
- `actor.GetActorScale3D()` → `{"x": float, "y": float, "z": float}`.
- `actor.SetActorScale3D(NewScale3D={"X": 1.0, "Y": 1.0, "Z": 1.0})` → None.
- `actor.GetActorBounds(bOnlyCollidingComponents=False, as_dict=True)` → `{"Origin": {"x":.., "y":.., "z":..}, "BoxExtent": {"x":.., "y":.., "z":..}}`. `BoxExtent` values are half-extents.
- `actor.K2_GetRootComponent()` → Root component.

### Component UFUNCTIONs

- `comp.SetMobility(NewMobility="Movable")` → None. Values: `"Static"`, `"Stationary"`, `"Movable"`. Call before moving or editing a statically-authored actor.
- `comp.SetStaticMesh(NewMesh=mesh_object)` → bool. Call `SetMobility(NewMobility="Movable")` first.
- `comp.SetMaterial(ElementIndex=0, Material=mat)` → None. Per-instance override for one slot.
- `comp.GetMaterial(ElementIndex=0)` → Material.
- `comp.GetNumMaterials()` → int.

### `USpotLightComponent` (analogous helpers exist for `UPointLightComponent`, `URectLightComponent`)

- `spot.SetIntensity(NewIntensity=3000.0)` → None.
- `spot.SetAttenuationRadius(NewRadius=500.0)` → None.
- `spot.SetOuterConeAngle(NewOuterConeAngle=30.0)` → None.
- `spot.SetInnerConeAngle(NewInnerConeAngle=10.0)` → None.

### Logging

- `spear.log("message")` → Print to MCP tool output.
