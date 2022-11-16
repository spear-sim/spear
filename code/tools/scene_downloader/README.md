# Scene Downloader
1. Run scene_downloader.py to download virtual worlds to `Content/`.

   ```bash
       python scene_downloader.py -i <virtualworld-id> -v <version-info> -p <proxy_host:proxy_port> -o <ouputDir>
       # for example
       python scene_downloader.py -i 235554690 -v v7 true -o ./Saved/ -c pak_split_linux
   ```

* `-i`: optional virtualworld-id. All available id lists can be found in data/virtualworld-ids.json. If not specified,
  all virtualworld-ids will be processed sequentially.

* `-v`: required scene version in format of v{n}. Up-to-date version information can be found in
  dataset-repo-update.log.

* `-f`: if '-f true', when downloading, the existing assets will be overwritten. if not use -f, comparing local version
  information(MD5 in it) to remote version information and decide whether to download asset.

* `-p`: if you need to run this script behind a proxy, use this option. Use this format, `-p hostname:port`. Don't
  include `http or https` in your hostname. For example:
  ```bash
  python scene_downloader.py -i 235554690 -v v1 -d true -p hostname:port
  ```
* `-o`: optional. Content saved directory. Default saved directory is `Saved/<version>/<relative_file_path>`.
* `-c`: optional. Download scene data type: `pak_split_windows`, `pak_split_linux`, `pak_split_mac`.

### Rendering Mode

Unreal provide real time ray-tracing on Windows only, the rendering setting in `DefaultEngine.ini` need to be configured
differently in different rendering mode:

* RTX on Windows:
  ```
      [/Script/Engine.RendererSettings]
      r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange=True
      r.DefaultFeature.LightUnits=2
      r.SkinCache.CompileShaders=True
      r.RayTracing=True
  ```
* Rasterization with baked lights:
  ```
      [/Script/Engine.RendererSettings]
      r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange=True
      r.ReflectionCaptureResolution=1024
      r.DefaultFeature.LightUnits=2
      r.SkinCache.CompileShaders=True
      r.RayTracing=False
      r.CustomDepth=3
      r.SSGI.Enable=True
      r.GenerateMeshDistanceFields=True
      r.DistanceFieldBuild.Compress=True
  ```
