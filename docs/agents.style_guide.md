# Local Style First

- New files must include both copyright lines: `Copyright (c) 2025 The SPEAR Development Team` and `Copyright (c) 2022 Intel`.
- Treat nearby code as the canonical style; match naming, spacing, casing, and call patterns in the file being edited.
- Prefer minimal diffs: change only what is needed for the requested behavior. Check `git diff` against the base branch to verify minimality before presenting changes.
- Prefer simple, direct code over extra helpers or temporary variables unless they improve clarity.
- Prefer symmetric, uniform structure so each piece of code can be verified locally. Similar functions, layers, and code paths should have the same shape and handle the same cases in the same way, even when that means some redundancy (e.g., every UnrealService function that returns a value passes a `convert_func` to its EntryPointCaller, rather than some functions converting their results after the call).
- Prefer not to write helper functions that call into the SPEAR programming model (e.g., `spear.*` services, `UnrealObject` methods, `instance.begin_frame()`/`instance.end_frame()`) unless there is an especially good reason to do so, because it is hard to make it obvious if the function expects to be called inside a SPEAR transaction or not. Some functions in the `spear` library break this rule (e.g., `instance.step()` and `game.async_loading_service.wait_for_engine_idle()` must be called outside a transaction), but their contracts are explicit and obvious.
- Prefer `if condition: ... else: ...` (or `if (condition) { ... } else { ... }` in C++) over an early return followed by fallthrough code. Put the positive condition in the `if` branch (e.g., `if (ptr) { return f(*ptr); } else { return false; }`, not `if (!ptr) { return false; } return f(*ptr);`).
- Keep logging/noise low; do not add extra debug output unless requested.
- Prefer single-line calls when they comfortably fit and remain readable. When a call with keyword arguments doesn't fit on one line, put one argument per line.
- Omit explicit default arguments when behavior is unchanged.
- In arithmetic expressions, use no spaces around `*` and `/`, but use spaces around `+` and `-` (e.g., `a*b + c`, `x/2.0 + y`, `fov*math.pi/180.0`). This applies to both Python and C++ code, including inside comments that contain formulas.
- In Python list comprehensions, use spaces inside the brackets (e.g., `[ x for x in y ]` not `[x for x in y]`).
- When calling Unreal editor Python API methods (e.g., `set_editor_property`, `get_editor_property`), always use named keyword arguments (e.g., `set_editor_property(name="width", value=100)` not `set_editor_property("width", 100)`).
- Prefer division by `2.0` over multiplication by `0.5`.
- When working with transforms in this repo, prefer `to_numpy_array_from_vector(..., as_matrix=True)` and matrix multiplication style.
- For transform setup variables, use ordering `location`, then `rotation`, then `scale` when practical.
- For per-axis scaling in matrix code paths, prefer a diagonal scale matrix (e.g., `np.diag(...)`) and matrix multiplication.
- Avoid redundant conversion chains (e.g., `as_matrix=True` followed immediately by `.A1`).
- Prefer direct `*` operations over `np.multiply(...)` for NumPy arrays.
- When using `np.matrix`, prefer `*` for matrix multiplication instead of `@`.
- In numpy dimension-expanding indexing, prefer `np.newaxis` over `None` (e.g., `x[:,:,np.newaxis]`, not `x[:,:,None]`).
- When implementing UFUNCTION/UPROPERTY declarations in C++, use Unreal naming conventions for parameter names (e.g., `DeltaTime`, `NewLocation`). Local variables inside function bodies use snake_case (e.g., `level_editor_viewport_client`). When overriding non-UFUNCTION virtual methods, preserve the method name capitalization from the base class but replace parameter names with our naming conventions (e.g., `graph_builder` not `GraphBuilder`).
- When calling Unreal UFUNCTIONs, use keyword argument names with exact Unreal casing.
- When calling `spear.*` functions, always use keyword arguments, even for single-argument calls (except variadic helpers like `entry_point_caller.call_on_game_thread`).
- `as_dict=True` returns a `spear.CaseInsensitiveDict` whose top-level keys match the UFUNCTION parameter names, ignoring case (plus `"ReturnValue"` for non-void returns). All nested dicts (e.g., struct values, structs inside containers, and `TMap` values) are also returned as `spear.CaseInsensitiveDict`s, because every dict returned by a service has string keys, and Unreal already treats these keys case-insensitively (e.g., `TMap<FString, ...>` and `TMap<FName, ...>` keys can't differ only by case). Keys that are FNames (e.g., struct property names) might differ in case from C++ outside the editor (see the `Unreal::getUnrealName(...)` comments in `Unreal.h`).
- In SPEAR Python code, specify Unreal enums as unqualified strings (e.g., `"DoubleBuffered"` not `"ESpBufferingMode::DoubleBuffered"`).
- Do not use type annotations in Python code (no `def f(x: int) -> str:`, no variable annotations).
- In Python, name module-level constants in lower case like any other variable, never in upper case (e.g., not `MACOS_DEPLOYMENT_TARGET`). In utils and services, a constant that is accessed outside its module must be exposed through a function instead (e.g., `spear.tools.get_macos_deployment_target()`, `spear.get_identity_transform()`), and a constant that is only used inside its module must have a leading underscore (e.g., `_windows_log_format`).
- In Python classes, name a member without a leading underscore (e.g., `self.config`) if anything outside the class accesses it, including derived classes, and with a leading underscore (e.g., `self._config`) if nothing outside the class accesses it. For example, `spear.Service` stores `self.config` because derived services access it, whereas `spear.Instance` stores `self._config`. Check which name the class you're editing actually defines rather than copying from a different class.
- Python services are either low-level or high-level. Low-level services typically have a corresponding C++ service, and may define `call_async` and `send_async` child services. They call RPC entry points only in their corresponding C++ service, although they may call Python helper functions in other services (e.g., `shared_memory_service.validate_arrays(...)`), and they never drive frames. High-level services are typically Python-only, don't define `call_async` or `send_async`, and implement helper functions by calling into low-level services or other high-level services. High-level services may optionally drive frames (e.g., `async_loading_service`).
- In argparse, do not pass `description=` to `ArgumentParser()` or `help=` to `add_argument()`. Use `assert args.x in [...]` after `parse_args()` instead of `choices=[...]`.
- In Python, group related values with dicts that have named keys (e.g., `{"title": title, "image": image}`) rather than tuples.
- Prefer `if-return-else-return` over `if-return-return` (always use explicit `else` for the alternate branch).
- Do not include message text in `assert` statements (use bare `assert condition`, not `assert condition, "message"`).
- In Python type-dispatch if-else chains, place the most general/catch-all branches last. Unless there is a good reason to deviate, order the remaining branches as: Python builtin scalar types (e.g., `bool`, `numbers.Integral`, `str`), then NumPy types, then SPEAR types (e.g., `spear.SpHandle`, `spear.UnrealObject`), then `list`, then `dict` (e.g., `spear.to_handle(...)`).
- In Python type-dispatch if-else chains, use one type per branch, even when bodies are identical. Merging types into one branch is allowed only if every type is a Python builtin (e.g., `bool`, `numbers.Number`, `str`, `list`, `dict`) or the shared body is non-trivial.
- In `__init__.py` files, keep imports alphabetical per line, grouped by category.
- Use `spear.path.realpath(...)` for constructing file paths. It combines `os.path.realpath(...)` and `os.path.join(...)`, so paths are always absolute. Use `os.path.join(...)` only when a path must stay relative or must not resolve symlinks, and add a comment explaining why.
- Functions in `spear.path` operate on filesystem paths by default. A function that operates on Unreal content paths (e.g., `/Game/Maps/MyMap`) must say so in its name (e.g., `get_filesystem_path_from_content_path(...)`, `get_content_name_from_filesystem_name(...)`).
- When running a subprocess, always log the command before executing it.
- When writing a file (e.g., an image), log the output path.
- Practice strict IWYU (Include What You Use) in C++ files. Every file includes exactly what it needs directly — no relying on transitive includes, not even between a .cpp file and its own header. The test is whether the file uses a symbol that is declared in the header. For example, if a file declares a member of type `FDelegateHandle`, it must include `<Delegates/IDelegateInstance.h>` — even if another included header happens to pull it in transitively. Conversely, if a file never mentions `FDelegateHandle` by name (e.g., the type is only used internally by a base class), the include is not needed.
- Header-only implementations are fine and often preferred. Template-heavy code must live in headers anyway, and many UCLASSes (especially thin wrappers in SpUnrealTypes) are header-only because keeping the declaration and implementation together is easier to maintain, and their headers are mostly included only by UHT-generated code, so they don't increase compile times much. Don't suggest moving these implementations into .cpp files.
- In C++ files, organize includes in groups separated by blank lines, in this order: (1) self-header (.cpp only, first), (2) C standard library (`<stdint.h>`), (3) C++ standard library (`<array>`, `<chrono>`, `<memory>`), (4) Unreal Engine headers (`<Components/...>`, `<RHI...>`), (5) SPEAR headers (`"SpCore/..."`, `"SpServices/..."`), (6) local module headers (`"SpUnrealTypes/..."`), (7) generated header (`"*.generated.h"`, header files only, very last). Sort alphabetically by full include path (case-insensitive) within each group. Inline comments on includes listing non-obvious symbol names should be sorted alphabetically. Whether to add a comment depends solely on how non-obvious the symbol is (e.g., `<utility>` needs `// std::move` because that mapping is non-obvious; `<string>` does not need `// std::string`). See the **Include Comment Alignment** section below for the column-alignment rule.
- In C++ headers, preserve and update comments documenting thread-access patterns on member variables. Inline comments on groups of related member variables should be aligned to the same column (one space after the longest declaration **that has a comment** in the group), using the same alignment principle as include comments.
- Guard C++ code with the correct preprocessor macro: use `#if WITH_EDITOR` for editor functionality (e.g., `IsRunningCommandlet()`, editor-only virtual overrides, editor subsystem access), and `#if WITH_EDITORONLY_DATA` for editor-only data (e.g., `GIsEditor`, `EditorStartupMap`). These are different macros with different scopes — `WITH_EDITOR` is a strict subset of `WITH_EDITORONLY_DATA`.
- When adding a new RPC entry point, order the lambda parameters to match the order they are used in the function body. For example, if the implementation calls `callFunction(world, uobject, ...)`, the lambda should take `uint64_t& world, uint64_t& uobject, ...` in that order.
- When adding a new custom struct type to the RPC interface, plumb it through all layers: `SpCore/SpTypes.h` or `SpServices/SpTypes.h` (server and client structs, in `SP_PLATFORM_SERVER`/`SP_PLATFORM_CLIENT` blocks, or a single `SP_PLATFORM_SERVER || SP_PLATFORM_CLIENT` block if the definitions are identical, and likewise with `SP_PLATFORM_UNREAL`/`SP_PLATFORM_PURE` in SpCore), `SpServices/MsgpackAdaptors.h` (server and client serialization — client `pack` + `object_with_zone` to send args, server `convert` to receive args, server `object_with_zone` to send return values, client `convert` to receive return values, in that order), `SpServices/FuncSignatureRegistry.h` (type name, server `getTypeId`, and client cases in `getArg` and `getReturnValue`), `python_ext/cpp/spear_ext.cpp` (nanobind class bindings), and `editor_utils.py` (Python ClientStruct + `_to_return_values` case). Only add the serialization directions that are actually needed — `object_with_zone` + `pack` for types sent as args, `convert` for types received as return values.
- Field ordering must be consistent across all serialization layers. The source of truth is the underlying C++ struct (e.g., `SpTypes.h`). `MsgpackAdaptors.h` matches the struct order (with any derived fields placed at the end of the adaptor snippet), and all downstream layers — the client struct, the client adaptors in `MsgpackAdaptors.h`, `python_ext/cpp/spear_ext.cpp` (nanobind `def_rw` order), and `editor_utils.py` (Python class `__init__` parameter and attribute order) — must follow the same field order. This consistency makes cross-layer review tractable: mismatches between layers become visually obvious without reasoning about set equality.
- In Python code that accepts a flexible input type (e.g., a UClass that could be a string name, a handle, or an `UnrealClass` object), prefer normalizing once via an existing converter (e.g., `_unreal_service.to_uclass()`, `spear.to_handle()`) rather than type-switching with `isinstance` at each call site.
- When creating a new general-purpose utility function, immediately scan all related existing functions to see if they can be simplified to thin wrappers over the new function. Do this in the same edit pass — don't wait to be asked. For example, if you create `convert_transform(transform, as_numpy, as_spear, ...)`, then `to_numpy_transform_from_spear_transform` and `to_spear_transform_from_numpy_transform` should immediately become one-line delegations to it.
- When citing function or symbol names in documentation or style-guide examples, grep first to confirm they actually exist in the codebase. Do not invent illustrative names — they mislead readers, break grep, and erode trust in the docs.
- In `SpServices/FuncSignatureRegistry.h`, type IDs are grouped as: primitive types, quasi-primitive types (e.g., `uint64_t`), simple std types, container std types, custom types. Keep all of the type tables (`s_type_names_`, `getTypeId`, `getArg`, `getReturnValue`) in sync with the same IDs and ordering.
- In C++ code, use same-line (Egyptian) braces for control flow statements (`if (...) {`, `for (...) {`, `else {`).
- Avoid ternary operators (`?:`) in C++ code unless there is a very good reason; prefer explicit `if`/`else`.
- For C++ map/set operations, prefer `Std::containsKey` over `.contains()`, `Std::insert` over `[]` assignment, and `Std::remove` over `.erase()`. These wrappers include assertions that catch common bugs (e.g., `Std::insert` asserts the key does not already exist, `Std::remove` asserts the key exists). Following this convention avoids the `[]` operator, which silently inserts a default value if the key is missing. Use `.at()` for lookups.
- For C++ container teardown, use `clear()` alone; do not call `shrink_to_fit()` (it is only a non-binding request).
- In C++ code, use `SP_ASSERT` for all assertions, including conditions that could be checked at compile time (e.g., `SP_ASSERT(sizeof(T) == sizeof(U))`, `SP_ASSERT(std::is_trivially_copyable<T>())`, `SP_ASSERT(std::atomic_ref<uint64_t>::is_always_lock_free)`). Do not use `static_assert`, except where a runtime assertion is not possible (e.g., at class or namespace scope).
- Prefer aggregate assignment for `std::array` initialization and teardown (e.g., `readback_buffers_ = { make_unique(...), nullptr }`) over per-element `.at(i) = ...` or `.at(i).reset()`.
- Suffix C++ functions that execute on the render thread with `_RenderThread` (e.g., `copyPixelsFromStagingToCPU_RenderThread`).
- For C++ `enum class` declarations, assign explicit integer values with aligned `=` signs (e.g., `Idle = 0`, `Pending = 1`). See `EngineService.h` for reference.
- When adding an `_in_editor_script` generator variant of an existing method, use `yield` only as pause points (after each `begin_frame`/`end_frame` context block) and `return` for the value; use `yield from` to delegate to sub-generators. If the regular and generator variants share an obvious long prefix and/or suffix, then factor it into shared `_impl` helpers rather than duplicating it. But if the differences between the variants are too interleaved, so factoring would only lead to a tiny amount of savings, then don't break up the behavioral locality of a 20-line function for a 2-line shared prefix.
- When adding a new buffering/readback mode, keep the same SpFunc interface (`enqueue_read_pixels` + `read_pixels`) and express mode differences through priming frame count and internal dispatch. In C++ if-else chains checking `BufferingMode`, order branches as SingleBuffered, DoubleBuffered, TripleBuffered.
- Prefer generic byte buffers (`std::vector<uint8_t>` with `SpAlignedAllocator`) and `SpArrayDataTypeUtils::getSizeOf` over typed containers with per-type if-else chains for pixel data.
- When a synchronous code path (e.g., single-buffered readback with `FlushRenderingCommands`) can write directly to the final output buffer, avoid intermediate copies through a scratchpad.
- For render target texture formats in asset creation scripts, match `num_channels_per_pixel` and texture format to the actual data dimensionality (e.g., `RTF_R16F`/`RTF_R8` for scalar modalities, not `RTF_RGBA16F`/`RTF_RGBA8`).
- In Python visualization lambdas for image data, explicitly index channels (e.g., `data[:,:,[0,1,2]]` for RGB, `data[:,:,[0,0,0]]` for greyscale) rather than passing arrays that may include alpha.
- Keep lists of component descriptors sorted alphabetically by name.
- Never remove existing diagnostic logging (spin-wait counts, warnings, etc.) without being explicitly asked. Diagnostic messages are critical for understanding runtime behavior.
- Only add functions to `SpCore/Std.h` when they provide a more ergonomic way to use the C++ standard library (e.g., `Std::contains(vector, value)` instead of `std::ranges::find(vector, value) != vector.end()`). Don't wrap functions that are already ergonomic. For example, use `std::string::starts_with` and `ends_with` directly.
- Don't put `SP_ASSERT` inside lambdas in range pipelines (e.g., `std::views::filter`, `std::views::transform`). Assert before the pipeline instead (e.g., `SP_ASSERT(Std::all(...))` or `SP_ASSERT(!Std::contains(vector, nullptr))`), so the pipeline lambdas contain only filtering and transformation logic.
- Write C++ comments as full sentences, wrapped at roughly 120 columns. When editing the text of an existing comment, rewrap the whole paragraph rather than leaving short or ragged lines behind. Let a line run past 120 columns to keep a long literal or example (e.g., `(e.g., "Meshes/05_chair/LivingRoom_Chair_01:StaticMeshComponent0")`) on the same line as the text that introduces it, rather than wrapping just before it.
- Mark every override of a virtual function with `override`, including destructors (e.g., `~WorldRegistryService() override`, `InitializeEngineService::postEngineInit() override`).
- Don't declare pure virtual functions in UCLASSes, because the engine must be able to construct every UCLASS (e.g., its CDO). Instead, give the function a body that asserts and returns a default value (e.g., `virtual ProxyComponentManagerBase* getProxyComponentManager() { SP_ASSERT(false); return nullptr; }` in `ASpProxyComponentManager`). Plain C++ classes that are owned by a UCLASS hierarchy and mirror its structure follow the same pattern for consistency (e.g., `MeshProxyComponentManager::shouldProxyComponentBeHiddenInViewport(...)`).
- When a UCLASS hierarchy owns a parallel hierarchy of plain C++ classes, expose the owned object through a virtual getter on the UCLASS base class that returns a non-template base class pointer, and override the getter in derived UCLASSes with a covariant (i.e., more derived) return type (e.g., `ASpProxyComponentManager::getProxyComponentManager()` returns `ProxyComponentManagerBase*`, and `ASpMeshProxyComponentManager` overrides it to return `MeshProxyComponentManager*`). This way, base class code can call the owned object's common interface, and derived class code can call its derived interface without casting. Note that a UCLASS can't be a template, and its first base class must be a non-template UObject class.
- In CRTP base classes that `static_cast` `this` to the derived type, make the default constructor private and declare the derived type as a friend, so only the intended derived class can construct the base class and the `static_cast` is guaranteed to be valid (e.g., `ProxyComponentManager<...>`). Don't use `dynamic_cast`, because SPEAR modules are compiled without RTTI.
- Group a class's member functions by interface, and keep each interface together even if some of its functions could move to a base class (e.g., the `request*()` functions in `ProxyComponentManager<...>`). Start each group with a short section comment that says what the group is for and where it is called from (e.g., `// ProxyComponentManagerBase interface. Called from ASpProxyComponentManager::initializeImpl() and ...`).
- Don't call `std::move` on const objects (e.g., elements of a `const std::map&` in a range-based for loop). It silently copies, which is misleading. Close member function bodies with `}`, not `};`.
- In C++ code, use `static_cast` rather than C-style casts. The one exception is a UPROPERTY array dimension sized by an enum (e.g., `ShaderPlatformForFeatureLevel[(int)ESpRHIFeatureLevel::Num]` in `SpRHIGlobals.h`), because UHT only recognizes C-style casts when detecting enum-sized arrays, and it needs that to label each array element with the corresponding enum name. Add a comment explaining this when you use the exception.
- When an SpUnrealTypes struct mimics an engine struct, derive sizes and constants from the engine (e.g., `UE_ARRAY_COUNT(GRHIGlobals.DefaultMSAASampleOffsets)` in `SpRHIGlobals.h`) rather than duplicating literals, and don't invent named constants that don't exist in the engine.
- Keep engine names when mimicking engine functions, even when the engine's names are confusing. Conversely, in utility classes that operate on engine types, reserve the `Get` prefix for functions that mimic an engine function named `Get...`. For example, `USpQualityLevelsUtils::QualityLevelsFromSingleQualityLevel(...)` has no `Get`, but `GetSingleQualityLevel(...)` mimics `FQualityLevels::GetSingleQualityLevel()`, and `USpScalability::GetQualityLevels()` mimics `Scalability::GetQualityLevels()`.
- Thin wrappers that expose engine functionality (e.g., static UFUNCTIONs in a `UBlueprintFunctionLibrary`) should expose the engine's behavior, not make it safer. Only `SP_ASSERT` pointers that the wrapper itself dereferences. Pass other pointers straight through to the engine without asserting, because `nullptr` might be a valid input to the engine function. Likewise, return the engine function's return value to the caller rather than asserting on it (e.g., `USpMovieSceneEventUtils::BindNewUserFacingEventForKey(...)` returns the `UK2Node*`).
- SPEAR validates pointers for core types that appear in SPEAR entry points (e.g., UObjects via `SpHandle`, `FProperty`, `IConsoleVariable`). Long-tail UCLASSes (e.g., in SpUnrealTypes or user plugins) may return unvalidated pointers as `uint64` (e.g., `USpSceneCaptureComponent2D::GetViewStates()`). In this case, don't add validation. Instead, each UCLASS should document its own volatility contract in a comment on the declaration (e.g., the returned pointers are not guaranteed to remain valid across a `begin_frame()` or `end_frame()` block and should be consumed immediately). Caching such pointers beyond their documented lifetime is a user error.
- When a UFUNCTION isn't BlueprintCallable because its signature or return type contains `uint64`, say so with a trailing comment on the declaration line: `static FSpRHIGlobals GetRHIGlobals() // uint64 not supported for BlueprintCallable`. This format is easy to find with text search.
- When aligning `=` signs in a block of consecutive assignments, align each blank-line-separated block independently, with `=` one space after the longest left-hand side in that block (e.g., `USpRHIGlobalsInterface::GetRHIGlobals()` in `SpRHIGlobals.h`).
- Setup and teardown pairs (`Initialize()`/`Terminate()`, `BeginPlay()`/`EndPlay()`, `subscribeToUserInputs(...)`/`unsubscribeFromUserInputs(...)`) must be symmetric: teardown undoes everything setup did, in reverse order, with matching conditions, and clears all saved state (e.g., `ASpInitializeWorldManager`, `USpSceneCaptureComponent2D`). For long lists of peer objects, it is fine to tear down the objects within a group in creation order so the setup and teardown lists are easy to read and keep in sync, as long as the groups themselves are torn down in reverse order (e.g., `SpServices::terminate()`).
- When setup and teardown functions are chained through a class hierarchy, follow constructor and destructor order: an override of a setup function calls the base class implementation first, and an override of a teardown function calls the base class implementation last (e.g., `ProxyComponentManager<...>::initialize()`/`terminate()`, `ASpMeshProxyComponentManager::initializeImpl()`/`terminateImpl()`). If a base class defines the chain but has nothing to do, give it empty implementations anyway (e.g., `ProxyComponentManagerBase::initialize()`), so every override can call its base class and the chain is obvious.
- Add Unreal engine module dependencies to `SpModuleRules` or `SpModuleRulesEditor`, but add plugin module dependencies only to the Build.cs files of the modules that use them, because any plugin module listed in `SpModuleRules` or `SpModuleRulesEditor` must also be listed in all of our uplugin files. Note that returning an engine type from a UFUNCTION requires linking against the module that defines the type, not just including its header (e.g., returning `UK2Node*` from `USpMovieSceneEventUtils` requires `BlueprintGraph`).
- Don't use `TWeakObjectPtr` in UFUNCTION signatures or Python-visible UPROPERTYs. Use raw pointers (e.g., `AActor*`) instead, because raw pointers are SPEAR's handle type for UObjects, UHT doesn't support weak pointers as BlueprintCallable parameters, and `UnrealUtils` only converts `FObjectProperty`s (not `FWeakObjectProperty`s) to and from strings. Internally, when storing a pointer to a UObject that we don't own and that might be destroyed while we're holding it, use `TWeakObjectPtr`. If the user explicitly passed the object into a stateful system (e.g., the actor passed to `ASpActorHitManager::SubscribeToActor(...)`), then the user is responsible for removing it from the system before destroying it, so call `Unreal::toPtr(...)`, which returns `nullptr` if the pointer isn't valid, and `SP_ASSERT(...)` the result. Otherwise (e.g., the other actor in a hit event), skip the object if `Unreal::toPtr(...)` returns `nullptr` (e.g., `SpActorHitManager.cpp`). Only resolve weak pointers (e.g., with `Unreal::toPtr(...)`, `IsValid()`, or `Get()`) on the game thread, because doing so reads engine-owned object state. Therefore, UObject pointers that are serialized, or cached somewhere that is read on the worker thread (usually because the data will be serialized), are stored as an `SpHandle`, created with `UnrealUtils::toHandle(...)` on the game thread and validated with `UnrealUtils::toPtr(...)` on the game thread (e.g., `SpWorldDesc::world_`, which `WorldRegistryService` returns from `"world_registry_service.get_world_descs"` and which therefore gets serialized on the worker thread, and `SpPropertyDesc::notify_objects_`). In summary: serialized UObject pointers are `SpHandle`s; UObject pointers cached in C++ are `TWeakObjectPtr`s, unless they're read on the worker thread, in which case they're `SpHandle`s.
- Prefer less state. For example, rather than caching pointers to engine objects that can be fetched where they're needed, fetch them in each function so there is no state to keep in sync (e.g., `USpUserInputComponent` fetches the player controller's `InputComponent` and `PlayerInput` in each function).
- In UCLASSes, internal bookkeeping state that only needs to be a UPROPERTY for garbage collection should be `UPROPERTY(Transient)`, with no visibility specifiers, so it is hidden in the editor (e.g., the `Saved*` members in `SpSceneCaptureComponent2D.h`). Runtime state mirrored for debugging should be `UPROPERTY(Transient, VisibleAnywhere, ...)`. Mark it `Transient` even if it only ever exists in the game world, as documentation.
- Singleton manager actors with user-facing configuration, buttons, or debug state must be placed in the map and looked up by name (e.g., `"Debug/ActorHitManager"` in `SpDebugManager.cpp`), rather than being driven through their CDO.
- When a class declares a destructor but needs no custom destructor logic, declare it as `= default` (e.g., `~USpUserInputComponent() = default;`).
- When SPEAR reimplements engine behavior, mimic the engine's implementation as closely as possible, and search the UE source (see `docs/agents.user.md`) to confirm how the engine does it. For example, the JSON import/export code in `SpCore/UnrealUtils.cpp` mimics `JsonObjectConverter.cpp`: it iterates sets and maps with `FScriptSetHelper::FIterator`/`FScriptMapHelper::FIterator`, deduplicates set elements and map keys the way the engine's `ImportText_Internal` does (import into temporary storage, then add only if not already present), and accepts the engine's precision limits rather than working around them.
- Prefer DRY in general, but use judgement: sometimes locality is a good reason to repeat a small amount of code. For example, the convenience overloads of `UnrealUtils::resolveProperty(...)` all delegate to a single implementation (DRY), whereas the two main `resolveProperty(...)` overloads (taking `FProperty*` and `const std::string&`) each repeat the same `parent` validation block so that each function can be read top to bottom on its own (locality). It is fine to explain design decisions in code comments next to the code they describe, rather than in separate docs.
- When parsing numbers from strings in C++, use `std::from_chars` and `SP_ASSERT` that it succeeded, that it consumed the whole string, and that the result is in range (e.g., array index parsing in `UnrealUtils::resolveProperty(...)`). Don't use `atoi` and related functions, which silently return 0 on invalid input.
- If unsure between two styles, follow the dominant pattern already present in the touched file.
- Do not add comments that suppress linter or tool warnings (e.g., `# noqa`, `# type: ignore`, `# pylint: disable`). If the code has a real problem, fix it. If the warning is only relevant to your tooling, silently ignore it — do not leave a trace in the source.
- When in doubt how to interpret any convention listed here, look at the existing source code, which should be 99% consistent already and is the canonical reference.

# Technical Writing

These conventions apply to long explanatory comments (e.g., the overview at the top of `SpServices/EngineService.h`) and to docs.

- Write for a smart computer science graduate student who has read the SPEAR paper (https://arxiv.org/abs/2607.06701) and the README but has never seen this codebase. Terms that the paper or the README introduces (e.g., the game thread, the worker thread, `instance.begin_frame()`) don't need to be introduced again. Read your text in order, the way that reader would, and keep track of what the reader knows at each sentence. Introduce every term (e.g., "entry point", "wrapper lambda", "transaction") before using it, and then use it consistently. Don't mention a concept, function, or entry point that the text hasn't introduced yet.
- An overview comment exists to help the reader make sense of the code it sits above. It isn't a user guide, so don't add usage instructions (e.g., how to enable a config option).
- When connecting the code to user-facing behavior, describe the path that users actually take, not an API that almost no one calls directly (e.g., "the server's error state is reset when the user's Python code creates a new instance", not "call instance.engine_service.initialize()").
- Start with the purpose and the big picture (e.g., "The primary responsibility of the SpServices plugin is to expose C++ functionality to Python."), then explain how the pieces fit together. Don't describe function-level details that are obvious from the code.
- Use definite language. Verify every claim against the code (including the UE source) before writing it, and don't hedge.
- State consequences and motivation explicitly (e.g., "Therefore, ...", "This design implies that ...", "This is maximally convenient, because ...").
- Refer to things by their precise names. Quote entry points by their full names (e.g., `"engine_service.begin_frame"`), write C++ functions with their class (e.g., `EngineService::bindFuncToExecuteOnGameThread(...)`), and use consistent noun phrases for the actors in the system (e.g., "the user's Python code", "the SPEAR Python library", "the worker thread", "the game thread").
- Give concrete examples in parentheses with "e.g.,", and use "i.e.," for clarifications.
- Use aligned bullet lists for parallel items, even if the aligned lines exceed 120 columns. Separate the sections of a long comment with short headings on their own line (e.g., `// Entry points`).
- When editing someone else's prose, fix only typos and formatting unless asked to do more, and suggest any other changes rather than making them.

# Include Comment Alignment

Within each include group, `//` comments must be vertically aligned. The alignment column is determined by the longest `#include` line **that has a comment** — not the longest include overall. The `//` starts exactly one space after the end of that longest commented include. All other commented lines in the group are padded with spaces to match.

If a new commented include becomes the longest in the group, update all other commented lines in the group to match the new column.

Don't add include comments that only name the header's eponymous type (e.g., `#include <string> // std::string`). Leave such includes uncommented. Don't add include comments to SPEAR headers (e.g., `#include "SpCore/UnrealUtils.h"`), except to explain something non-obvious (e.g., `#include "SpCore/Windows.h" // On Windows, rpclib expects <windows.h> to have been included`).

The same alignment principle applies to inline comments on member variable declarations (see the bullet on thread-access pattern comments above).

**Algorithm:**
1. Within a single include group, find all `#include` lines that will have a `//` comment.
2. Measure the character length of the longest such `#include` directive (everything up to and including the closing `>` or `"`).
3. The `//` column (0-indexed) = that length + 1 (one space).
4. Pad every other commented `#include` line with spaces so its `//` starts at the same column.
5. Uncommented includes are left as-is — they do not participate in the alignment calculation.

To avoid mental arithmetic errors, compute padding with:
```python
col = max(len(inc) for inc in commented_includes) + 1
padded = inc + " " * (col - len(inc)) + "// " + comment
```

To verify alignment after editing, use `awk index()` which returns a **1-indexed** column number:
```bash
sed -n '11,13p' file.h | awk '{print NR": // at col "index($0, "//")}'
```
All commented lines in the group must report the same column. The expected column (1-indexed) = `len(longest_commented_include) + 2` (one for the space, one for the 1-indexing offset).

**Example** (from `SpCore/UnrealUtils.h`):

```cpp
// C++ standard library group
#include <concepts> // std::derived_from
#include <map>
#include <ranges>   // std::views::drop, std::views::filter, std::views::transform
#include <string>
#include <utility>  // std::make_pair, std::pair
#include <vector>

// Unreal Engine group
#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/Array.h>
#include <Containers/UnrealString.h> // FString::operator*
#include <Dom/JsonValue.h>
#include <EngineUtils.h>             // TActorIterator
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>            // SPCORE_API
#include <Templates/Casts.h>
#include <Templates/SharedPointer.h> // TSharedPtr
#include <UObject/Class.h>           // EIncludeSuperFlag
#include <UObject/Object.h>          // UObject
#include <UObject/ObjectMacros.h>    // EPropertyFlags
#include <UObject/Script.h>          // EFunctionFlags
#include <UObject/UnrealType.h>      // EFieldIterationFlags, FProperty, TFieldIterator
#include <UObject/UObjectGlobals.h>  // NewObject
#include <UObject/UObjectIterator.h>
```

In the C++ standard library group, the longest commented include is `#include <concepts>` (19 characters), so `//` starts at 0-indexed column 20. `<ranges>` and `<utility>` are shorter and padded with spaces to align. `<map>`, `<string>`, and `<vector>` have no comments and are not padded.

In the Unreal Engine group, the longest commented includes are `#include <Containers/UnrealString.h>` and `#include <Templates/SharedPointer.h>` (both 36 characters), so `//` starts at 0-indexed column 37. Every other commented include in the group (`<EngineUtils.h>`, `<HAL/Platform.h>`, `<UObject/UnrealType.h>`, etc.) is padded to match that same column. Uncommented includes like `<Components/ActorComponent.h>` and `<UObject/UObjectIterator.h>` are left as-is even though some are longer than the longest commented include.

# Cross-Platform Compiler Checks

When a C++ change uses a language feature whose portability is uncertain (e.g., explicit specializations at class scope, constrained templates, or other C++20 features), and the relevant platform isn't available locally, check it against the relevant compilers using Compiler Explorer's public REST API. Compile a minimal stand-alone repro that isolates the feature in question rather than sending SPEAR source code. To list compiler IDs (e.g., `vcpp_v19_latest_x64` for MSVC):

```bash
curl -s -H "Accept: application/json" https://godbolt.org/api/compilers/c++
```

To compile a file (`code` is 0 on success, and compiler diagnostics are returned in `stderr` and `stdout`):

```python
import json, urllib.request
body = json.dumps({"source": open("repro.cpp").read(), "options": {"userArguments": "/std:c++20 /EHsc /W4"}}).encode()
request = urllib.request.Request("https://godbolt.org/api/compiler/vcpp_v19_latest_x64/compile", data=body, headers={"Content-Type": "application/json", "Accept": "application/json"})
result = json.load(urllib.request.urlopen(request))
print(result["code"], [line["text"] for line in result["stderr"] + result["stdout"]])
```

Test a few compiler versions that span the range we support, and treat a passing repro as evidence rather than proof; the full build on the target platform is still the source of truth.
