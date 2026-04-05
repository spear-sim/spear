# Local Style First

- New files must include both copyright lines: `Copyright (c) 2025 The SPEAR Development Team` and `Copyright (c) 2022 Intel`.
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
- Practice strict IWYU (Include What You Use) in C++ files. Every file includes exactly what it needs directly — no relying on transitive includes, not even between a .cpp file and its own header. The test is whether the file uses a symbol that is declared in the header. For example, if a file declares a member of type `FDelegateHandle`, it must include `<Delegates/IDelegateInstance.h>` — even if another included header happens to pull it in transitively. Conversely, if a file never mentions `FDelegateHandle` by name (e.g., the type is only used internally by a base class), the include is not needed.
- In C++ files, organize includes in groups separated by blank lines, in this order: (1) self-header (.cpp only, first), (2) C standard library (`<stdint.h>`), (3) C++ standard library (`<array>`, `<chrono>`, `<memory>`), (4) Unreal Engine headers (`<Components/...>`, `<RHI...>`), (5) SPEAR headers (`"SpCore/..."`, `"SpServices/..."`), (6) local module headers (`"SpUnrealTypes/..."`), (7) generated header (`"*.generated.h"`, header files only, very last). Sort alphabetically by full include path (case-insensitive) within each group. Inline comments on includes listing non-obvious symbol names should be sorted alphabetically. Whether to add a comment depends solely on how non-obvious the symbol is (e.g., `<utility>` needs `// std::move` because that mapping is non-obvious; `<string>` does not need `// std::string`). See the **Include Comment Alignment** section below for the column-alignment rule.
- In C++ headers, preserve and update comments documenting thread-access patterns on member variables. Inline comments on groups of related member variables should be aligned to the same column (one space after the longest declaration **that has a comment** in the group), using the same alignment principle as include comments.
- Guard C++ code with the correct preprocessor macro: use `#if WITH_EDITOR` for editor functionality (e.g., `IsRunningCommandlet()`, editor-only virtual overrides, editor subsystem access), and `#if WITH_EDITORONLY_DATA` for editor-only data (e.g., `GIsEditor`, `EditorStartupMap`). These are different macros with different scopes — `WITH_EDITOR` is a strict subset of `WITH_EDITORONLY_DATA`.
- When adding a new RPC entry point, order the lambda parameters to match the order they are used in the function body. For example, if the implementation calls `callFunction(world, uobject, ...)`, the lambda should take `uint64_t& world, uint64_t& uobject, ...` in that order.
- When adding a new custom struct type to the RPC interface, plumb it through all layers: `SpTypes.h` (C++ server struct), `MsgpackAdaptors.h` (server serialization), `FuncSignatureRegistry.h` (server type ID), `python_ext/cpp/types.h` (client struct), `python_ext/cpp/msgpack_adaptors.h` (client serialization — `convert` for receive, `pack` + `object_with_zone` for send), `python_ext/cpp/func_signature_registry.h` (client type ID + cases in `getArg` and `getReturnValue`), `python_ext/cpp/spear_ext.cpp` (nanobind class bindings), and `editor_utils.py` (Python ClientStruct + `_to_return_values` case). Only add the serialization directions that are actually needed — `object_with_zone` + `pack` for types sent as args, `convert` for types received as return values.
- In `SpServices/FuncSignatureRegistry.h` and `python_ext/cpp/func_signature_registry.h`, type IDs are grouped as: primitive types, quasi-primitive types (e.g., `uint64_t`), simple std types, container std types, custom types. Keep both files in sync with the same IDs and ordering.
- In C++ code, use same-line (Egyptian) braces for control flow statements (`if (...) {`, `for (...) {`, `else {`).
- Avoid ternary operators (`?:`) in C++ code unless there is a very good reason; prefer explicit `if`/`else`.
- For C++ map/set operations, prefer `Std::containsKey` over `.contains()`, `Std::insert` over `[]` assignment, and `Std::remove` over `.erase()`. These wrappers include assertions that catch common bugs (e.g., `Std::insert` asserts the key does not already exist, `Std::remove` asserts the key exists). Following this convention avoids the `[]` operator, which silently inserts a default value if the key is missing. Use `.at()` for lookups.
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
- Do not add comments that suppress linter or tool warnings (e.g., `# noqa`, `# type: ignore`, `# pylint: disable`). If the code has a real problem, fix it. If the warning is only relevant to your tooling, silently ignore it — do not leave a trace in the source.
- When in doubt how to interpret any convention listed here, look at the existing source code, which should be 99% consistent already and is the canonical reference.

# Include Comment Alignment

Within each include group, `//` comments must be vertically aligned. The alignment column is determined by the longest `#include` line **that has a comment** — not the longest include overall. The `//` starts exactly one space after the end of that longest commented include. All other commented lines in the group are padded with spaces to match.

If a new commented include becomes the longest in the group, update all other commented lines in the group to match the new column.

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
