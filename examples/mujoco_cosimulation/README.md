# Unreal-Mujoco Co-Simulation

1. setup spear dev branch

```
   git clone --recurse-submodules https://github.com/spear-sim/spear.git
   cd spear 
   git checkout --rwang15/mujoco-cosim
```

2. install spear python(see getting_started.md)

```console
   # create environment
   conda create --name spear-env python=3.11
   conda activate spear-env

   # install msgpack-rpc-python separately from other Python dependencies, because we need
   # to use a specific commit from a specific fork of the msgpack-rpc-python GitHub repository
   pip install -e third_party/msgpack-rpc-python

   # install the spear Python package
   pip install -e python
```

3. config Mujoco co-simulation example:

   Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents
   appropriately for your system, as described in our [Getting Started](../../docs/getting_started.md) tutorial.
   Your `user_config.yaml` file only needs to specify the value of a parameter if it differs from the defaults defined
   in the `python/spear/config` directory. You can browse this directory for a complete set of all user-configurable
   parameters.

   Typically, change `SPEAR.STANDALONE_EXECUTABLE` to standalone executable path

4. run_sceme_cosimulation.py --mjcf_file <path_to_scene_mjcf>

### TODO list

* more accurate collision
    * Hand-crafted collision if provided
    * better CoACD parameter, shrink to mesh
* collision
* instruction for adding more agents
* unreal agent in mujoco
* read camera data in python from unreal
