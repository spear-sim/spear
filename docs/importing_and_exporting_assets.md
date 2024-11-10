# Importing and Exporting Assets

Our approach for importing and exporting assets is to formulate each task (e.g., import from dataset X, export to destination application Y, etc) as a pipeline consisting of modular stages. More specifically, each pipeline is a directed acyclic graph of stages, and each stage is a Python program that produces and/or consumes data in a user-specified top-level directory. Each stage can optionally access functionality available in the Unreal Editor via a Python interface.

## Assumptions

In order to execute the pipelines in this document, we will assume that you have completed our [Getting Started](getting_started.md) and [Building SpearSim](building_spearsim.md) tutorials. We will also assume that you want to execute all pipelines for the `apartment_0000` scene only, and that you want all pipeline output to be generated in a top-level directory called `spear-pipeline`.

## Accessing the Unreal Editor via Python

In order to access the Unreal Editor via Python, we must first configure the editor's Python environment, even though we have already configured an Anaconda Python environment in our [Getting Started](getting_started.md) tutorial.

```console
python tools/configure_editor_python_env.py --unreal_engine_dir path/to/UE_5.4
```

Any pipeline stage that needs to access the Unreal Editor must be executed using our `run_editor_script.py` tool, which runs a user-specified program (specified by `--script`) from within the editor's Python environment. `run_editor_script.py` consumes `--script` and `--unreal_engine_dir`, and forwards all other arguments directly to the user's program. `--script` must be relative to `spear/pipeline` or absolute. Any path arguments that are forwarded to the user's program must be absolute.

## Using our debug visualization tools

In order to use our optional debug visualization tools, you will need to install an appropriate [TraitsUI](https://docs.enthought.com/traitsui/#installation) backend for your platform.

## Pipelines

### Export to MuJoCo

```console
# generate Unreal metadata
python tools/run_editor_script.py --script export_unreal_metadata/export_unreal_metadata.py --unreal_engine_dir path/to/UE_5.4 --pipeline_dir /absolute/path/to/spear-pipeline --scene_id apartment_0000

# generate Unreal geometry
python tools/run_editor_script.py --script export_unreal_metadata/export_unreal_geometry.py --unreal_engine_dir path/to/UE_5.4 --pipeline_dir /absolute/path/to/spear-pipeline --scene_id apartment_0000

# visualize Unreal geometry (optional)
python pipeline/visualize_unreal_geometry.py --pipeline_dir path/to/spear-pipeline --scene_id apartment_0000

# generate a compact kinematic tree scene representation
python pipeline/generate_kinematic_trees.py --pipeline_dir path/to/spear-pipeline --scene_id apartment_0000

# visualize the compact kinematic tree scene representation (optional)
python pipeline/visualize_kinematic_trees.py --pipeline_dir path/to/spear-pipeline --scene_id apartment_0000

# generate optimized collision geometry
python pipeline/generate_collision_geometry.py --pipeline_dir path/to/spear-pipeline --scene_id apartment_0000

# visualize the collision geometry (optional)
python pipeline/visualize_collision_geometry.py --pipeline_dir path/to/spear-pipeline --scene_id apartment_0000

# generate a MuJoCo scene (ignoring the ceiling actor is optional but makes the scene easier to visualize)
python pipeline/generate_mujoco_scene.py --pipeline_dir path/to/spear-pipeline --scene_id apartment_0000 --ignore_actors Meshes/22_ceiling/Ceiling

# interactively browse the MuJoCo scene using the default MuJoCo viewer (optional)
python -m mujoco.viewer --mjcf=path/to/spear-pipeline/scenes/apartment_0000/mujoco_scene/main.mjcf
```
