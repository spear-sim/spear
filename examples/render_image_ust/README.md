# Render an Image (UserSceneTexture)

In this example application, we demonstrate how to read UserSceneTexture buffers from a UST camera sensor. Each capture pass of `BP_CameraSensor_UST` and `BP_CameraSensorPathTracer_UST` carries the full "universe" of post-process material instances (`MI_PPM_*_UST`) with the enabled set empty; this example enables the whole universe on a single pass, renders once, and saves the main capture and every UST buffer as separate images.

```console
python run.py --blueprint path_tracer --component lighting_only_
```
