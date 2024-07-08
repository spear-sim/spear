# Running the Export Pipeline

## Assumptions

We will assume that you have completed our [Getting Started](getting_started.md) and [Building SpearSim](building_spearsim.md) tutorials. We will also assume that we want to run the export pipeline for the `apartment_0000` scene.

## Configure the Unreal Editor Python environment

The Unreal Editor ships with its own Python environment that can be used to run scripts from inside the editor. Some of our pipeline steps must be run from inside the editor, and therefore we must configure the editor's Python environment, even though we have already configured an Anaconda Python environment in our [Getting Started](getting_started.md) tutorial. 

```console
python tools/configure_editor_python.py --unreal_engine_dir path/to/UE_5.2
```

## Run each pipeline stage

Our export pipeline consists of several modular stages that can be executed sequentially as follows. In this example, each pipeline stage will generate its output in a directory called `spear-pipeline` that we specify using the `--pipeline_dir` command-line argument.

In order to use our optional debug visualization tools below, you will need to install an appropriate [TraitsUI](https://docs.enthought.com/traitsui/#installation) backend for your platform.


```console
# generate Unreal metadata
python tools/run_editor_script.py --unreal_engine_dir path/to/UE_5.2 --script generate_unreal_metadata.py --pipeline_dir path/to/spear-pipeline --scene_id apartment_0000

# generate Unreal geometry
python tools/run_editor_script.py --unreal_engine_dir path/to/UE_5.2 --script generate_unreal_geometry.py --pipeline_dir path/to/spear-pipeline --scene_id apartment_0000

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
python pipeline/generate_collision_geometry.py --pipeline_dir path/to/spear-pipeline --scene_id apartment_0000 --ignore_actors Meshes/22_ceiling/Ceiling

# interactively browse the MuJoCo scene using the default MuJoCo viewer
python -m mujoco.viewer --mjcf=path/to/spear-pipeline/apartment_0000/mujoco_scene/main.mjcf
```
