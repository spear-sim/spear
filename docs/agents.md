# SPEAR Programming Model

See `README.md` for a full introduction with code examples.

- SPEAR is a Python library that programmatically controls Unreal Engine applications. Any C++ function or variable visible to UE's reflection system (`UFUNCTION`/`UPROPERTY`) can be called as native Python.
- `spear.Instance` represents a connection to a UE application. `instance.get_game()` returns scoped services for the game world (PIE); `instance.get_editor()` returns scoped services for the editor world. In a standalone game, only `instance.get_game()` is available and calling `instance.get_editor()` is an error.
- Graphs of UE work are specified as transactions via `begin_frame`/`end_frame` context blocks. Every `with instance.begin_frame():` **must** be followed by exactly one `with instance.end_frame():` — they always come in pairs. Code in `begin_frame` executes at the beginning of a single UE frame; code in `end_frame` executes at the end of the same frame. Side effects are immediately observable within the same block — e.g., spawning an actor and then calling a function on it in the same `begin_frame` works.
- UFUNCTIONs are called with keyword arguments using exact Unreal casing (e.g., `actor.SetActorScale3D(NewScale3D=...)`). UPROPERTYs are read via `.get()` (e.g., `actor.RootComponent.get()`) and written via direct assignment (e.g., `component.Width = 1024`).
- Scoped services (`game`/`editor`) provide `unreal_service` for spawning, finding, and destroying actors, and `get_unreal_object()` for obtaining Python wrappers around UE objects.
- `instance.step(num_frames=N)` advances N frames (useful for letting rendering, physics, or temporal effects settle).

# Design Principles

SPEAR is a single-maintainer research codebase. The following are deliberate design decisions, not oversights. Don't flag them as problems in code reviews or assessments.

- The RPC server uses a single worker thread. This is a feature: it allows user Python code and server code to be reasoned about as though they were a single thread, even though they execute in different threads (and usually different processes).
- Assertions are used for error handling (`SP_ASSERT` in C++, bare `assert` in Python), including for validating user input. Don't suggest replacing them with exceptions.
- There is no unit test suite. The programs in `examples/` serve as end-to-end tests. Don't suggest adding tests.
- Some definitions are manually kept in sync across multiple source files (e.g., RPC types across the server, the `spear_ext` client, and `editor_utils.py`; see `docs/agents.style_guide.md`). This keeps the build simple, and these definitions change infrequently. Don't suggest code generation.
- Code is grouped by locality of behavior and concern, so some files are large. Large files are fine as long as they don't mix unrelated concerns. In a few places (e.g., `UnrealService_0.h` and `UnrealService_1.h`), code is split across files to reduce compile times, even though this makes it harder to browse.
- The pure-Python `editor_utils.Client` doesn't call `SharedMemoryFence::fence()` because it runs inside the same process as the server, so there is no interprocess memory visibility to enforce.
