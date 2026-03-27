# Style Guide Sync

`CLAUDE.md` and `.cursor/rules/local-style.mdc` are the two style guides for this repo and must always be kept in sync. Any change to one must be reflected in the other.

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
- When implementing UFUNCTION/UPROPERTY declarations in C++, use Unreal naming conventions for parameter and variable names (e.g., `DeltaTime`, `NewLocation`). When overriding non-UFUNCTION virtual methods, preserve the method name capitalization from the base class but replace parameter names with our naming conventions (e.g., `graph_builder` not `GraphBuilder`).
- When calling Unreal UFUNCTIONs, use keyword argument names with exact Unreal casing.
- When calling `spear.*` functions, always use keyword arguments, even for single-argument calls (except variadic helpers like `entry_point_caller.call_on_game_thread`).
- For `as_dict=True` Unreal returns, access keys exactly as named in the call/UE signature.
- In SPEAR Python code, specify Unreal enums as unqualified strings (e.g., `"DoubleBuffered"` not `"ESpBufferingMode::DoubleBuffered"`).
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
