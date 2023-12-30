# Open-Loop Control with Fetch

In this example application, we demonstrate how control a Fetch agent to pick up an object and move it to another location.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our [Getting Started](../../docs/getting_started.md) tutorial.

### Running the example

You can run the example as follows.

```console
# generate actions
python generate_actions.py

# execute actions
python run.py
```

Running `generate_actions.py` will generate an `actions.csv` file consisting of actions that will be used in the following step. This tool accepts an optional `--actions_file` command-line argument that can be used to specify the file that will be generated.

Running `run.py` will execute the previously generated actions in an open-loop fashion on the Fetch agent. This tool accepts several optional command-line arguments that can be used to control its behavior (see the source code for details), e.g.,

  - `--actions_file` can be used to specify which actions to load.
