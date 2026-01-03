# Getting Started (in the Unreal Editor)

In this example editor script, we demonstrate how to how to spawn a simple object and access some of the object's properties. If you're working in the Unreal Editor, you can run this script from the editor's Python console as follows. See [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/scripting-the-unreal-editor-using-python) for details.

```console
/absolute/path/to/spear/examples/getting_started_editor/run.py
```

Alternatively, you can run this script from the command-line as follows.

```console
python ../../tools/run_editor_script.py --script /absolute/path/to/spear/examples/getting_started_editor/run.py --unreal-engine-dir path/to/UE_5.5 --launch-mode full --render-offscreen --save-level
```

After running this script, you should see a set of coordinate axes appear in the scene immediately, or the next time you open the editor, depending on how you invoked the script.
