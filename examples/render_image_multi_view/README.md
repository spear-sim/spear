# Render Multiple Views

In this example application, we demonstrate how to spawn a multi-view camera sensor object and use it to render multiple images. You can run this example as follows.

```console
python run.py
```

You should see a game window appear.  Several images will be saved to an `images` directory, which can be combined into a video with `ffmpeg` as follows:

```console
ffmpeg -i "images/%d.png" video.mp4
```
