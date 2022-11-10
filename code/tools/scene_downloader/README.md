
# Virtual World

### Scene Manager

1. Run SceneManager/scene_manager.py to download virtual worlds to `Content/`.

	```bash
        python scene_manager.py -i <option virtualworld-id> -v <necessary version-info> -d <option is_download_ddc> -p <proxy_host:proxy_port> -o <ouputDir>
        # for example
        python scene_manager.py -i 235554690 -v v4 -d true -o ./Saved/
	```

   -i: optional virtualworld-id. All available id lists can be found in SceneManager/Data/virtualworld-ids.json. If not specified, all virtualworld-ids will be loaded.
   
   -v: required scene version in format of v{n}. Up-to-date version information can be found in SceneManager/dataset-repo-update.log.
   
   ```
	v1: RTX method, the renderersettings in ./RobotProject/Config/DefaultEngine.ini should be as follow:
      [/Script/Engine.RendererSettings]
      r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange=True
      r.DefaultFeature.LightUnits=2
      r.SkinCache.CompileShaders=True
      r.RayTracing=True
   v2: Baking method, the renderersettings in ./RobotProject/Config/DefaultEngine.ini should be as follow:
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
   
   -d: default false, whether download ddc. See [UE4 DerivedDataCache](https://docs.unrealengine.com/4.26/en-US/ProductionPipelines/DerivedDataCache/) for more information.  Modify .\Epic Games\UE_4.26\Engine\Config\BaseEngine.ini as bellow:

	```
	[InstalledDerivedDataBackendGraph]
	Local=(Type=FileSystem, ReadOnly=false, Clean=false, Flush=false, PurgeTransient=true, DeleteUnused=true, nusedFileAge=34, FoldersToClean=-1, Path="%GAMEDIR%DerivedDataCache", EditorOverrideSetting=LocalDerivedDataCache)
	```

   -f: if '-f true', when downloading, the existing assets will be overwritten. if not use -f, comparing local version information(MD5 in it) to remote version information and decide whether to download asset.

   -p: if you need to run this script behind a proxy, use this option. Use this format, `-p hostname:port`. Don't include `http or https` in your hostname.
	With proxy, command would look something like:
	```bash
	python scene_manager.py -i 235554690 -v v1 -d true -p hostname:port

	```
   `-o`: optional. Content saved directory. Default saved directory is `Saved/<version>/<relative_file_path>`.
   `-c`: optional. Download scene data for different cases. If not specified, download scene data for Windows. Valid content types: `pak_split_windows`, `pak_split_linux`, `pak_split_mac`.
   ```bash
   python scene_manager.py -i 235554690 -v v4 -c panorama
   ```

2. If download fails or there are materials missing in Virtual World (mostly due to internet issues), try run 'scene_manager.py -v v1 -f true -i <virtualworld-id>' to reload the scene. Download log can be found in `Saved/UpdateLog/{virtualworld-id}_failed.txt`.
3. Start from v4, SceneManger delivers virtual worlds in `.pak` format which can be used directly in standalone executable without scene cooking step. Currently, it only supports Linux.

### Scene Manager Metadata
1. download basic metrics by following script
```
python scene_manager_meta.py -v <version> -i <virtual-world-id>  -p <proxy>
# e.g.
python scene_manager_meta.py -v v2
```
* `-v`: required. VirtualWorld version in format of v{n}, e.g. v4. The latest version information is in /VirtualWrold/SceneManager/dataset-repo-update.log.  
* `-i`: optional. Specify to download metadata for VirtualWorld. if not use -i, the script will load all virtualworld-ids in /VirtualWrold/SceneManager/Data/virtualworld-ids.json.
* `-p`: optional. setup proxy

### Parse Metadata
parse all available meta.json and output statistics result: sum and average based on style
```
scene_manager_meta_parser.py -k <keys>
# e.g: 
python scene_manager_meta_parser.py --keys "room,asset,Living room"
 ```

* -k: required. comma separated key list. Available key: 
 room, area, asset, asset_unique, Living room, Bedroom, Bathroom, Kitchen, Aisle, Balcony, Study, Customize, Dinning room	

### Top View Images

Download top view images and semantic images use following script:

```
python scene_manager_images.py -v <version> -i <virtual-world-id>  -p <proxy>
# e.g.
python scene_manager_images.py -v v2
```

* `-v`: required. VirtualWorld version in format of v{n}, e.g. v1, v2. The latest version information is in
  /VirtualWrold/SceneManager/dataset-repo-update.log.
* `-i`: optional. Specify to download metadata for VirtualWorld. if not use -i, the script will load all
  virtualworld-ids in /VirtualWrold/SceneManager/Data/virtualworld-ids.json.
* `-p`: optional. setup proxy

There will be three file downloade:
* `top_view.png` : top view image.
* `top_view_semantic.png` : top view image in semantic mode. Semantic color mapping can be found in [link](https://docs.google.com/spreadsheets/d/1dtWZOmYr40xc2fasLo63aJB1KFXmuWoDXGafoAJHf6U/edit#gid=1058311551).
* `image_data.json` : listing information for coordinate conversion between image pixel position and unreal coordinates:

```
x_unreal = width / 2 - y_image + x_center
y_unreal = - width / 2 + x_image + y_center

x_image = width / 2 + y_unreal - y_center
y_image = width / 2 - x_unreal + x_center
```

### panorama
For each scene, we generate panorama images for five largest rooms for each scene, with camera placed in the center of each room. The images are in the order of the room size.
