# Importing and Exporting Assets

In general, our approach for importing/exporting assets to/from the Unreal Engine is to formulate each import/export task (e.g., import from dataset X, export to destination application Y, etc) as a pipeline consisting of modular stages. We typically implement each pipeline as a directed acyclic graph of stages, and each stage is a Python program that produces and/or consumes data in a user-specified top-level directory for a particular scene. Each stage can optionally access functionality available in the Unreal Editor using the SPEAR API or the editor's built-in Python API. In this document, we demonstrate some example export pipelines that are implemented in this style.

## Assumptions

In order to execute the pipeline in this document, we will assume that you have completed our [Getting Started](getting_started.md) tutorial. We will also assume that you want to execute the pipeline for the `apartment_0000` scene only, and that you want all pipeline output to be generated in a top-level directory called `spear-pipeline`.

## Installing additional Python dependencies

In order to execute the pipeline in this document, you will need to install the `pipeline` optional dependencies. When executing the command below, `PIP_BUILD_CONSTRAINT` forces `pip` to build against the versions we specify in `python/pip_build_constraint.txt`, and `--no-cache-dir` forces a fresh build.

```console
# install additional pipeline dependencies (Windows Powershell)
$env:PIP_BUILD_CONSTRAINT="python/pip_build_constraint.txt"; pip install --no-cache-dir -e "python[pipeline]"

# install additional pipeline dependencies (Windows Command Prompt)
set PIP_BUILD_CONSTRAINT=python/pip_build_constraint.txt && pip install --no-cache-dir -e "python[pipeline]"

# install additional pipeline dependencies (macOS and Linux)
PIP_BUILD_CONSTRAINT=python/pip_build_constraint.txt pip install --no-cache-dir -e "python[pipeline]"
```

## Accessing the Unreal Editor's built-in Python Interface

Any pipeline stage that needs to access the Unreal Editor using the editor's built-in Python API can be executed using our `run_editor_script.py` tool, which runs a user-specified script `--script` from within the editor's built-in Python environment. `run_editor_script.py` consumes `--script`, `--unreal-engine-dir`, and several optional arguments such as `--launch-mode` and `--render-offscreen`, and forwards all other arguments directly to the user's script. `--script` must be relative to `spear/pipeline` or absolute. Any path arguments that are forwarded to the user screen must be absolute.

## Maintaining Visual Parity with Unreal

When executing the pipelines below, specifying the optional `--visual-parity-with-unreal` flag will modify the positions and orientations of meshes to maintain visual parity with the Unreal viewport. This flag is necessary to account for the various coordinate conventions in different viewers.

## Exporting to MuJoCo

```console
# generate Unreal metadata
python tools/run_editor_script.py --unreal-engine-dir path/to/UE_5.5 --launch-mode full --render-offscreen --script export_unreal_metadata/run.py --export-dir path/to/spear-pipeline/scenes/apartment_0000

# generate Unreal geometry
python tools/run_editor_script.py --unreal-engine-dir path/to/UE_5.5 --launch-mode full --render-offscreen --script export_unreal_geometry.py --export-dir path/to/spear-pipeline/scenes/apartment_0000

# visualize Unreal geometry (optional)
python pipeline/visualize_unreal_geometry.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000 --visual-parity-with-unreal --ignore-actors Meshes/22_ceiling/Ceiling

# generate a compact kinematic tree scene representation
python pipeline/generate_kinematic_trees.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000

# visualize the compact kinematic tree scene representation (optional)
python pipeline/visualize_kinematic_trees.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000

# generate optimized collision geometry
python pipeline/generate_collision_geometry.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000

# visualize the collision geometry (optional)
python pipeline/visualize_collision_geometry.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000

# generate a MuJoCo scene
python pipeline/generate_mujoco_scene.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000 --mujoco-model-name apartment_0000 --visual-parity-with-unreal --ignore-actors Meshes/22_ceiling/Ceiling --color-mode unique_color_per_body

# interactively browse the MuJoCo scene using the default MuJoCo viewer (optional)
python -m mujoco.viewer --mjcf=path/to/spear-pipeline/scenes/apartment_0000/mujoco_scene/main.mjcf
```

## Generating Flythrough Videos

```console
# generate Unreal metadata
python tools/run_editor_script.py --unreal-engine-dir path/to/UE_5.5 --launch-mode full --render-offscreen --script export_unreal_metadata/run.py --export-dir path/to/spear-pipeline/scenes/apartment_0000

# generate Unreal geometry (only required for the optional visualization steps below)
python tools/run_editor_script.py --unreal-engine-dir path/to/UE_5.5 --launch-mode full --render-offscreen --script export_unreal_geometry.py --export-dir path/to/spear-pipeline/scenes/apartment_0000

# generate free-space points
python pipeline/generate_free_space_points.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000

# visualize the free-space points (optional)
python pipeline/visualize_free_space_points.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000 --visual-parity-with-unreal --ignore-actors Meshes/22_ceiling/Ceiling

# generate a visibility graph between free-space points
python pipeline/generate_free_space_visibility_graph.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000

# visualize the visibility graph (optional)
python pipeline/visualize_free_space_visibility_graph.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000 --visual-parity-with-unreal --ignore-actors Meshes/22_ceiling/Ceiling

# generate free-space bounding boxes that cover the free-space points
python pipeline/generate_free_space_bounding_boxes.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000

# visualize the free-space bounding boxes (optional)
python pipeline/visualize_free_space_bounding_boxes.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000 --visual-parity-with-unreal --ignore-actors Meshes/22_ceiling/Ceiling

# generate smooth paths through the free space
python pipeline/generate_free_space_paths.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000

# visualize the smooth paths (optional)
python pipeline/visualize_free_space_paths.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000 --visual-parity-with-unreal --ignore-actors Meshes/22_ceiling/Ceiling

# generate camera keyframes that include orientations along each path
python pipeline/generate_free_space_camera_keyframes.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000 --user-config-files path/to/user_config.yaml --view-selection-mode argmax

# visualize the camera keyframes (optional)
python pipeline/visualize_free_space_camera_keyframes.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000 --visual-parity-with-unreal --ignore-actors Meshes/22_ceiling/Ceiling

# generate camera paths that include orientations
python pipeline/generate_free_space_camera_paths.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000

# visualize the camera paths (optional)
python pipeline/visualize_free_space_camera_paths.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000 --visual-parity-with-unreal --ignore-actors Meshes/22_ceiling/Ceiling

# generate rendered images along each camera path
python pipeline/generate_free_space_camera_path_images.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000 --user-config-files path/to/user_config.yaml

# generate a rendered video for each camera path
python pipeline/generate_free_space_camera_path_videos.py --pipeline-dir path/to/spear-pipeline/scenes/apartment_0000
```
