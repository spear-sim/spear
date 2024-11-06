# Render an Image

In this example application, we demonstrate how to spawn a camera sensor object and use it to render images.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our [Getting Started](../../docs/getting_started.md) tutorial. Your `user_config.yaml` file only needs to specify the value of a parameter if it differs from the defaults defined in the `python/spear/config` directory. You can browse this directory for a complete set of all user-configurable parameters.

### Running the example

You can run this example as follows.

```console
python run.py
```

You should see a game window appear, as well as an OpenCV window that matches what you see in the game window. You can close the OpenCV window by pressing any key while it is in focus.
