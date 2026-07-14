# Render a Path-Traced Image

Renders a single image with a `USpSceneCaptureComponent2D` routed through Unreal's offline path tracer. Same flow
as [`render_image`](../render_image), but with the path tracer instead of the real-time renderer. The result is
saved to `image.png`.

## Usage

```cmd
python run.py
python run.py --teaser --num-frames 128 --bounces 12 --denoiser OIDN
```

- `--teaser` — render at 1920x1080 (otherwise the viewport resolution is used).
- `--num-frames` — number of frames to render while the path tracer accumulates samples (more frames, less noise).
- `--bounces` — maximum path-tracer bounces.
- `--denoiser` — spatial denoiser to apply, e.g. `NNEDenoiser` or `OIDN` (off by default).

## Before running

- A hardware ray-tracing capable GPU is required. `r.RayTracing.Enable=1` is already set in SpearSim's `DefaultEngine.ini` (it must be set at startup and can't be changed at runtime).
- Rename `user_config.yaml.example` -> `user_config.yaml`.
- The example config uses `LAUNCH_MODE:"none"` + `EDITOR_LAUNCH_MODE:"game"`: open the SpearSim project in the editor, enter play mode, then run `run.py`. The render uses the viewport camera and the current level.
