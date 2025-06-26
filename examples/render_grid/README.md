# Render an Image

In this example application, we demonstrate how to spawn several camera sensor objects in a grid formation and use it to render multiple images. You can run this example as follows.

```console
python run.py
```

You should see a game window appear.  Several files named `montage_{idx}.png` will be saved to an `images` directory. These can be combined into a video with `ffmpeg` as:

```console
ffmpeg -i "images/montage_%d.png" montage.mp4
```
