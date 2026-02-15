# MuJoCo Interoperability

In this example application, we demonstrate how to synchronize SPEAR rendering with an interactive MuJoCo simulation of our `apartment_0000` scene.

Before running this example, you will need to run our MuJoCo export pipeline for the `apartment_0000` scene, as described in our [Importing and Exporting Assets](../../docs/importing_and_exporting_assets.md) tutorial. After completing this tutorial, you can run the example as follows.

```console
python run.py --mjcf-file path/to/spear-pipeline/scenes/apartment_0000/mujoco_scene/main.mjcf --visual-parity-with-unreal
```

This will launch an interactive MuJoCo viewer and a SPEAR game window, and the SPEAR window will be synchronized to the MuJoCo simulation in real-time.
