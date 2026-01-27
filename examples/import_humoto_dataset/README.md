# Import Humoto Dataset

In this example application, we demonstrate how to import a dataset of [Humoto](https://jiaxin-lu.github.io/humoto) animation sequences.

We will assume that you have created a top-level directory that contains one or more subdirectories that contain one or more FBX files that contain one or more self-contained animation sequences. For example, suppose you have downloaded the _drinking_from_mug1_and_talking_suit_ and _checking_floor_lamp_with_right_hand_suit_ animation sequences from the Humoto website into the following directory structure.

```
humoto_dataset
├── ...
├── checking_floor_lamp_with_right_hand_suit
│   └── checking_floor_lamp_with_right_hand_suit.fbx
├── drinking_from_mug1_and_talking_suit
│   └── drinking_from_mug1_and_talking_suit.fbx
└── ...
```

With this directory structure in place, you can import the animation sequences into Unreal as follows.

```console
# import humoto dataset
python ../../tools/run_editor_script.py --script /absolute/path/to/spear/examples/import_animation_dataset/import_dataset.py --unreal-engine-dir path/to/UE_5.7 --launch-mode full --render-offscreen --filesystem-base-dir /absolute/path/to/humoto_dataset --content-base-dir /Game/Humoto
```

The `user_config.yaml.example` file in this example sets `SPEAR.LAUNCH_MODE` to `"none"`, which means the example is intended to run directly within a live game session in the Unreal Editor. If you launch the editor and press the play button, you can run spawn the previously imported animation sequences within the live game session as follows.

```console
# spawn animation sequences
python run.py
```

The _drinking_from_mug1_and_talking_suit_ and _checking_floor_lamp_with_right_hand_suit_ animation sequences should appear directly in the editor viewport. It is expected that they will disappear when you exit the live game session by pressing the stop button in the editor.
