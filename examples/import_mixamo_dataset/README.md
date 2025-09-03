# Import Mixamo Dataset

In this example application, we demonstrate how to import a dataset of [Mixamo](https://www.mixamo.com) animation sequences.

We will assume that you have created a top-level directory that contains a one or more subdirectories that contain one or more FBX files that contain one or more animation sequences. For example, suppose you have downloaded the _Crouch to Stand_ and _Start Plank_ animation sequences from Mixamo into the following directory structure.

```
mixamo_dataset
├── ...
├── Crouch To Stand
│   └── Crouch To Stand.fbx
├── Jump
│   └── Jump.fbx
└── ...
```

With this directory structure in place, you can import the animation sequences into Unreal as follows.

```console
# import mixamo dataset
../../tools/run_editor_script.py --script /absolute/path/to/spear/examples/import_animation_dataset/import_dataset.py --unreal_engine_dir path/to/UE_5.5 --launch_mode full --render_offscreen --animation_dataset_raw_base_dir /absolute/path/to/mixamo_dataset --animation_dataset_content_base_dir /Game/Mixamo
```

This example has `SPEAR.LAUNCH_MODE` set to `"none"`, which means the example is intended to run directly inside the Unreal Editor. If you launch the Unreal Editor and press the play button, you can run spawn the previously imported animation sequences as follows.

```console
# spawn animation sequences directly in a live game session in the editor
run.py
```

The _Crouch To Stand_ and _Jump_ animation sequences should appear directly in the editor viewport. It is expected that they will disappear when you press the stop button in the Unreal Editor.
