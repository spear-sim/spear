# MuJoCo Interoperability

In this example application, we demonstrate how to synchronize SPEAR rendering with an interactive MuJoCo simulation of our `apartment_0000` scene.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our [Getting Started](../../docs/getting_started.md) tutorial. You will also need to run our export pipeline for the `apartment_0000` scene, as described in our [Running the Export Pipeline](../../docs/running_export_pipeline.md) tutorial. 

### Running the example

You can run the example as follows.

```console
python run.py --mjcf_file path/to/spear-pipeline/apartment_0000/mujoco_scene/main.mjcf
```

This will launch an interactive MuJoCo viewer and a SPEAR rendering window. The poses of the chairs in the MuJoCo simulation will be synchronized in real-time with SPEAR.
