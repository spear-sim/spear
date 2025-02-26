# Render an Image

In this example application, we demonstrate how to spawn several camera sensor objects in a grid formation and use it to render multiple images.

Before running this example, rename `user_config.yaml.example` to `user_config.yaml` and modify the contents appropriately for your system, as described in our [Getting Started](../../docs/getting_started.md) tutorial. Your `user_config.yaml` file only needs to specify the value of a parameter if it differs from the defaults defined in the `python/spear/config` directory. You can browse this directory for a complete set of all user-configurable parameters.

### Running the example

You can run this example as follows.

```console
python run.py
```

You should see a game window appear.  Several files named `montage_{idx}.png` will be saved to your PWD.  These can be comebined into a video with ffmpeg as:
```
ffmpeg -i "montage_%d.png" montage.mp4
```
