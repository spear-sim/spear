# Importing and Exporting Assets

In general, our approach for importing/exporting assets to/from the Unreal Engine is to formulate each import/export task (e.g., import from dataset X, export to destination application Y, etc) as a pipeline consisting of modular stages. We typically implement each pipeline as a directed acyclic graph of stages, and each stage is a Python program that produces and/or consumes data in a user-specified top-level directory. Each stage can optionally access functionality available in the Unreal Editor using the SPEAR API or the editor's built-in Python interface. In this document, we demonstrate some example import/export pipelines that are implemented in this style.

## Assumptions

In order to execute the pipelines in this document, we will assume that you have completed our [Getting Started](getting_started.md) tutorial. We will also assume that you want to execute all pipelines for the `apartment_0000` scene only, and that you want all pipeline output to be generated in a top-level directory called `spear-pipeline`.

## Accessing the Unreal Editor's built-in Python Interface

Any pipeline stage that needs to access the Unreal Editor using the editor's built-in Python interface can be executed using our `run_editor_script.py` tool, which runs a user-specified script (specified by `--script`) from within the editor's built-in Python environment. `run_editor_script.py` consumes `--script` and `--unreal-engine-dir`, and forwards all other arguments directly to the user's script. `--script` must be relative to `spear/pipeline` or absolute. Any path arguments that are forwarded to the user's program must be absolute.

## Using our debug visualization tools

In order to use our optional debug visualization tools, you will need to install an appropriate [TraitsUI](https://docs.enthought.com/traitsui/#installation) backend for your platform.

## Pipelines

### Export to MuJoCo

```console
# generate Unreal metadata
python tools/run_editor_script.py --script export_unreal_metadata/run.py --unreal-engine-dir path/to/UE_5.5 --pipeline-dir /absolute/path/to/spear-pipeline --scene-id apartment_0000

# generate Unreal geometry
python tools/run_editor_script.py --script export_unreal_geometry.py --unreal-engine-dir path/to/UE_5.5 --pipeline-dir /absolute/path/to/spear-pipeline --scene-id apartment_0000

# visualize Unreal geometry (optional)
python pipeline/visualize_unreal_geometry.py --pipeline-dir path/to/spear-pipeline --scene-id apartment_0000

# generate a compact kinematic tree scene representation
python pipeline/generate_kinematic_trees.py --pipeline-dir path/to/spear-pipeline --scene-id apartment_0000

# visualize the compact kinematic tree scene representation (optional)
python pipeline/visualize_kinematic_trees.py --pipeline-dir path/to/spear-pipeline --scene-id apartment_0000

# generate optimized collision geometry
python pipeline/generate_collision_geometry.py --pipeline-dir path/to/spear-pipeline --scene-id apartment_0000

# visualize the collision geometry (optional)
python pipeline/visualize_collision_geometry.py --pipeline-dir path/to/spear-pipeline --scene-id apartment_0000

# generate a MuJoCo scene (ignoring the ceiling actor is optional but makes the scene easier to visualize)
python pipeline/generate_mujoco_scene.py --pipeline-dir path/to/spear-pipeline --scene-id apartment_0000 --ignore-actors Meshes/22_ceiling/Ceiling

# interactively browse the MuJoCo scene using the default MuJoCo viewer (optional)
python -m mujoco.viewer --mjcf=path/to/spear-pipeline/scenes/apartment_0000/mujoco_scene/main.mjcf
```
