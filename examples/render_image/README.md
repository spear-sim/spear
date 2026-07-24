# Render an Image

In this example application, we demonstrate how to spawn a camera sensor object and use it to render images. You can run this example as follows.

```console
python run.py
```

You should see a game window appear, as well as an OpenCV window that matches what you see in the game window. You can close the OpenCV window by pressing any key while it is in focus. Additionally, you should see an image saved in the same directory as this `README` file.

This application also includes an optional `--capture` flag, which will generate a GPU frame trace that can be analyzed in [Pix](https://dev.epicgames.com/documentation/unreal-engine/using-pix-on-windows-with-unreal-engine), [RenderDoc](https://dev.epicgames.com/documentation/unreal-engine/using-renderdoc-with-unreal-engine), or the [XCode](https://developer.apple.com/documentation/xcode/metal-debugger). In order for `--capture` to have any effect, you must install an appropriate GPU frame debugger from the list above, and enable that debugger your `user_config.yaml` file. If you're using the XCode, you must also explicitly enable the `XcodeGPUDebuggerPlugin` plugin in `SpearSim.uproject`.
