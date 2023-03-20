# Open Loop Control with UrdfBot

In this example application, we demonstrate how to load a Urdf agent and apply simple actions such as base motion, arm motion and gripper grasping.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our top-level [README](http://github.com/isl-org/spear).

### Running the example 

```console
python run.py --actions_file=actions.csv --observation_file=observations.csv
```

- `actions_file`: preconfigured joint actions.
- `observation_file`: store received link_position observation.