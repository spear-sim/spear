# Render an Image (Using Buffered Readback Modes)

In this example application, we demonstrate the latency characteristics of each buffering mode (single-buffered, double-buffered, triple-buffered) by spawning objects one per frame and verifying the object count in the returned pixels. You can run this example as follows.

```console
python run.py
```

You should see a game window appear, as well as a series of OpenCV windows showing the rendered images for each buffering mode. The window titles indicate the expected number of visible objects. You can close each OpenCV window by pressing any key while it is in focus.
