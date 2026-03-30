# SPEAR Programming Model

See `README.md` for a full introduction with code examples.

- SPEAR is a Python library that programmatically controls Unreal Engine applications. Any C++ function or variable visible to UE's reflection system (`UFUNCTION`/`UPROPERTY`) can be called as native Python.
- `spear.Instance` represents a connection to a UE application. `instance.get_game()` returns scoped services for the game world (PIE); `instance.get_editor()` returns scoped services for the editor world. In a standalone game, only `instance.get_game()` is available and calling `instance.get_editor()` is an error.
- Graphs of UE work are specified as transactions via `begin_frame`/`end_frame` context blocks. Code in `begin_frame` executes at the beginning of a single UE frame; code in `end_frame` executes at the end of the same frame. Side effects are immediately observable within the same block — e.g., spawning an actor and then calling a function on it in the same `begin_frame` works.
- UFUNCTIONs are called with keyword arguments using exact Unreal casing (e.g., `actor.SetActorScale3D(NewScale3D=...)`). UPROPERTYs are read via `.get()` (e.g., `actor.RootComponent.get()`) and written via direct assignment (e.g., `component.Width = 1024`).
- Scoped services (`game`/`editor`) provide `unreal_service` for spawning, finding, and destroying actors, and `get_unreal_object()` for obtaining Python wrappers around UE objects.
- `instance.flush(num_frames=N)` advances N frames (useful for letting rendering, physics, or temporal effects settle).
