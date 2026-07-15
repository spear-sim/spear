# Render a Path-Traced Image

In this example application, we demonstrate how to spawn a camera sensor object and use it to render images using the Unreal's path tracer.

```cmd
python run.py --num-frames 128 --bounces 12 --denoiser oidn
```

You should see a game window appear, as well as an OpenCV window showing the final output from the path tracer. You can close the OpenCV window by pressing any key while it is in focus. Additionally, you should see an image saved in the same directory as this `README` file.
