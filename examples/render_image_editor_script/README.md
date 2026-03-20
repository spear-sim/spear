# Render an Image (in the Unreal Editor using SPEAR)

In this example editor script, we demonstrate how to how to render an image. If you're working in the Unreal Editor, you can run this script from the editor's Python console as follows. See [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/scripting-the-unreal-editor-using-python) for details. Unlike the `render_image_editor` example, this script uses SPEAR functionality to do the do the rendering.

To run this example, execute the following command in the Unreal Editor's Python console:

```console
/absolute/path/to/spear/examples/render_image_editor_script/run.py
```

Alternatively, you can run this script from the command-line as follows.

```console
python ../../tools/run_editor_script.py --unreal-engine-dir path/to/UE_5.5 --launch-mode full --render-offscreen --script /absolute/path/to/spear/examples/render_image_editor_script/run.py
```

You should see a game window appear, as well as an OpenCV window that matches what you see in the game window. You can close the OpenCV window by pressing any key while it is in focus. Additionally, you should see an image saved in the same directory as this `README` file.
