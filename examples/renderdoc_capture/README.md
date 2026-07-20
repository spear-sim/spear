# Render an Image and capture a RenderDoc trace

In this example application, we demonstrate how to spawn a camera sensor object and use it to render images, and capture a detailed trace of the GPU commands in the process using RenderDoc. RenderDoc can be installed from https://renderdoc.org/. Please also adjust the paths in `user_config.yaml`.

```cmd
python run.py
```

You should see a game window appear, as well as a RenderDoc instance with the rendered frame capture. Please note some captures can get quite large, high-quality and high-resolution renders sometimes take up many gigabytes. Additionally, you should see an image saved in the same directory as this `README` file.