## SPEAR v0.2.0

This release includes the following new user-facing features and improvements.

- A large new photorealistic scene, `warehouse_0000`, which could be used to train industrial robots for warehouse applications.
- A new scene that is intended for debugging, `starter_content_0000`, which loads very quickly and is suitable for simple manipulation and navigation tasks.
- A new agent, Fetch, that is capable of mobile manipulation tasks.
- A new example application, `open_loop_control_fetch`, which demonstrates our Fetch agent performing a simple mobile manipulation task.
- A new tool, `run_executable`, which launches `SpearSim` in interactive navigation mode and loads a user-specified scene.
- Improved behavior when running the `SpearSim` executable with no additional arguments. The default behavior is now an interactive navigation mode, where a user can navigate around a default scene.
- An improved communication mechanism between the `SpearSim` executable and surrounding Python code, which more than doubles the overall frame rate when sending large images.
- An improved navigation policy that was trained using our `imitation_learning_openbot` example application to collect data.
- Our macOS `SpearSim` executable is now correctly code-signed.

## SPEAR v0.1.0

- Initial release.
