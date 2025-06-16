# Getting Started (in the Unreal Editor)

In this example editor script, we demonstrate how to how to spawn a simple object and access some of the object's properties. If you're working in the Unreal Editor, you can run this script from the editor's Python console as follows. See [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/scripting-the-unreal-editor-using-python) for detail.s

```console
/absolute/path/to/spear/examples/getting_started_editor/run.py
```

You should see a set of coordinate axes appearing immediately in the scene.

Alternatively, you can run this code from the command-line as follows, but you will need to un-comment the code in `run.py` to load and save the scene.

```console
python ../../tools/run_editor_script.py --script /absolute/path/to/spear/examples/getting_started_editor/run.py --unreal_engine_dir path/to/UE_5.5
```

After running this code from the command-line, you should see the coordinate axes in the scene the next time you open the editor.
