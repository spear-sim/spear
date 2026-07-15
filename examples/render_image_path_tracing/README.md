# Render a Path-Traced Image

In this example application, we demonstrate how to spawn a camera sensor object and use it to render images using the Path Tracer.

```cmd
python run.py
python run.py --teaser --num-frames 128 --bounces 12 --denoiser OIDN
```

You should see a game window appear, as well as an OpenCV window with the path tracer output image. You can close the OpenCV window by pressing any key while it is in focus. Additionally, you should see an image saved in the same directory as this `README` file.
