# Import Mixamo Dataset

In this example application, we demonstrate how to import a dataset of [Mixamo](https://www.mixamo.com) animation sequences.

We will assume that you have created a top-level directory that contains one or more subdirectories that contain one or more FBX files that contain one or more self-contained animation sequences. For example, suppose you have downloaded the _Crouch to Stand_ and _Jump_ animation sequences from the Mixamo website into the following directory structure.

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
python ../../tools/run_editor_script.py --unreal-engine-dir path/to/UE_5.5 --launch-mode full --render-offscreen --script /absolute/path/to/spear/examples/import_mixamo_dataset/import_dataset.py --filesystem-base-dir /absolute/path/to/mixamo_dataset --content-base-dir /Game/Mixamo
```

Next, we open the Unreal Editor and wait for the default map to fully load. Then we press play in the editor and wait for the Unreal simulation to load and warm up. Once the simulation is fully loaded and warmed up, we can run spawn the previously imported animation sequences within the live game session as follows.

```console
# spawn animation sequences
python run.py
```

The _Crouch To Stand_ and _Jump_ animation sequences should appear directly in the editor viewport. It is expected that they will disappear when you exit the live game session by pressing the stop button in the editor.
