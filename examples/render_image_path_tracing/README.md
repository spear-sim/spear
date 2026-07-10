# Render a Path-Traced Image

Renders a single path-traced image with a `USpSceneCaptureComponent2D`, without Movie Render Queue. Same flow
as [`render_image`](../render_image), plus the configuration to route the scene capture through the path tracer.

Usage:
```cmd
python run.py
python run.py --width 1920 --height 1080 --samples 256 --denoiser OIDN
```

## Before running

First you need a hardware ray-tracing capable GPU, and set `r.RayTracing.Enable=1` in `DefaltEngine.ini` before launching the editor. `r.RayTracing.EnableOnDemand=1` alone is not enough.

If you want to use the denoiser (`--denoiser OIDN` for example), you must also enable a plugin for each third-party denoiser. Go to *editor > Edit > Plugins* and type "denoise" to see all the plugins available (but not always enabled) by default.

The example `user_config.yaml` uses a `LAUNCH_MODE:"none"` + `EDITOR_LAUNCH_MODE:"game"` for convenience and fast iteration time.
To run the `run.py`, open the SpearSim project in the UE editor and enter play mode. The render uses the viewport camera and the current level.
