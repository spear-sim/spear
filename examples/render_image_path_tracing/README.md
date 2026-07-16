# Render a Path-Traced Image

In this example application, we demonstrate how to spawn a camera sensor object and use it to render images using the Unreal's path tracer.

```cmd
python run.py --num-frames 128 --num-bounces 12 --denoiser oidn
```

You should see a game window appear, some console output indicating rendering progress, you should see an image saved in the same directory as this `README` file. This example can only be run on platforms where Unreal supports Hardware Ray Tracing. See [here](https://dev.epicgames.com/documentation/unreal-engine/hardware-ray-tracing-in-unreal-engine) for details.
