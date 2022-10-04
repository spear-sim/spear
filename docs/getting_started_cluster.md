# Getting Started (Cluster)

"Cluster-HowTo" to rapidly get through the jungle of cuda and learning framework versions, thereby avoiding time-consuming incompatibility issues.
​
## Logging in and usage of Slurm cluster
​
- Start by logging on the cluster:
​
    ```
    ssh user@isl-iam1.rr.intel.com
    ```
​
- You should now be in one of the login nodes of the cluster. The golden rule is to avoid doing anything from this place that may potentially slow down users, i.e. no download, no computation etc… This should only be used for launching computing sessions.
- You have 2 distinct partitions:
  - `$HOME` is a fast 300Gb space for installing software
  - `$WORK` is a 1Tb space for storing data
  - We also have a shared space for the InteriorSim project in `/export/share/projects/InteriorSim/`, containing the .pak of each environment within the subfolder `/export/share/projects/InteriorSim/scene_manager_download_data/v4.`
- Open a slurm interactive session (i.e. with feedback in a terminal using the flag `--qos=inter --pty bash`) with access to couple of xeon CPUs (`-p cpu  -c 2`):
​
    ```
    srun -p cpu  -c 2 --qos=inter --pty bash
    ```
​
- This kind of sessions have the highest priority on the cluster but only have a 2h lifetime. They are ideal for installing something or rapidly debugging some learning algorithm but should not be used for large computations which may easily take several days.
​
## Installation
​
### Anaconda3
​
- Once logged into your slurm interactive session, the first step is to install anaconda, which will then allow you to load the required drivers and learning frameworks:
​
    ```
    wget https://repo.anaconda.com/archive/Anaconda3-2022.05-Linux-x86_64.sh
​
    sh Anaconda3-2022.05-Linux-x86_64.sh
    ```
​
- *Optionally*, update anaconda:
  - update the conda package manager to the latest version (may have to `source $HOME/.bashrc` first)
​
        ```
        conda update conda
        ```
  - use conda to update Anaconda to the latest version
​
        ```
        conda update anaconda
        ```
​
### Pytorch
​
The following shows how to set a pytorch environment, with access to GPU computing power:
​
- Use either an existing conda environment or create and activate your pytorch environment:
​
    ```
    conda create -n pytorch-env pip python=3.9
    conda activate pytorch-env
    ```
​
- Install cudatoolkit version 11.3.1:
​
    ```
    conda install -c anaconda cudatoolkit
    ```
​
- Install CUDNN version 8.2.1 (compatible with cudatoolkit 11.3.1)
​
    ```
    conda install -c anaconda cudnn
    ```
​
- Install the latest stable pytorch version (currently 1.12.1) using the nice command generation tool from <https://pytorch.org/>
​
    ```
    conda install pytorch torchvision -c pytorch
    ```
​
### Tensorflow
​
The following shows how to set a tensorflow environment, with access to GPU computing power:
​
- Use either an existing conda environment or create and activate your tensorflow environment:
​
    ```
    conda create -n tensorflow-env pip python=3.9
    conda activate tensorflow-env
    ```
​
- Install cudatoolkit version 11.3.1
​
    ```
    conda install -c anaconda cudatoolkit
    ```
​
- Install CUDNN version 8.2.1 (compatible with cudatoolkit 11.3.1)
​
    ```
    conda install -c anaconda cudnn
    ```
​
- Install tensorflow 2.5.2 using the pip
​
    ```
    pip install tensorflow==2.5.2
    ```
​
### Unreal Engine
​
Let's now take a look on how to build the Unreal Engine:
​
- I here assume that you already took the Required Setup (c.f. Step 1 in [Unreal Engine Documentation](https://docs.unrealengine.com/4.27/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/)) to get access to the UE4 git repo
- **Don't clone this repo**: it's HUGE. Download the 4.26 release instead. As follows
  - Access to the private GitHub repository needs to be requested by following Step 1 in [Unreal Engine Documentation](https://docs.unrealengine.com/4.27/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/), and by joining the EpicGames organisation (clicking on the email sent by them).
  - Once access to the UnrealEngine repository is granted, the link "https://github.com/EpicGames/UnrealEngine/archive/refs/tags/4.26.2-release.zip" should be accessible. To be able to curl/wget the link, a [personal access token](https://github.com/settings/tokens) needs to be generated on GitHub.
  - Then, the zip file can be downloaded by replacing `<personal_access_token>` with the personal access token generated from GitHub.
​
    ```
    cd $HOME
    mkdir UnrealEngine
    cd UnrealEngine
    curl -H "Authorization: token <personal_access_token>" -L https://github.com/EpicGames/UnrealEngine/archive/refs/tags/4.26.2-release.zip > 4.26.2-release.zip
    unzip 4.26.2-release.zip
    ```
​
- Installing cmake (sudo access is not provided on cluster): Follow instructions from [this link](https://pachterlab.github.io/kallisto/local_build.html). Steps:
  - Requirements: C++11 compatible compiler (>= g++-4.8), zlib, make
  - **Important**: It is assumed that all ThirdParty libraries are installed to `$HOME/ThirdParty`. The prefix argument will point to there and specifies where the binary files are located, ensure that it aligns with `PATH`/`LD_LIBRARY_PATH`.
  - We need to extend the paths (ideally, put this into `.bashrc`/`.zshrc`):
​
        ```
        export PATH=$HOME/ThirdParty/bin:$PATH
        export LD_LIBRARY_PATH=$HOME/ThirdParty/lib/:$LD_LIBRARY_PATH
        ```
  - download Cmake from [website](https://cmake.org/download/)
  - Install Cmake to the folder `$HOME/ThirdParty` (or any other path that was specified in prefix and where `PATH` includes it):
​
    ```
    tar -xf cmake*.tar.gz
​
    cd cmake*
​
    ./configure --prefix=$HOME/ThirdParty
​
    make
​
    make install
    ```
​
  - Ensure correct installation by running
​
    ```
    cmake --version
      ```
​
- It is also necessary to download `clang 11.0.0` to build UE4 on the cluster.
  - The most convenient is to download the prebuilt binaries:
​
    ```
    cd $HOME
    mkdir -p ThirdParty/clang
    cd ThirdParty/clang
    wget https://github.com/llvm/llvm-project/releases/download/llvmorg-11.0.0/clang+llvm-11.0.0-x86_64-linux-gnu-ubuntu-20.04.tar.xz
    tar -xzvf clang+llvm-11.0.0-x86_64-linux-gnu-ubuntu-20.04.tar.xz
    mv clang+llvm-11.0.0-x86_64-linux-gnu-ubuntu-20.04/* $HOME/ThirdParty/
    ```
​
  - Note: if not done in the `cmake` step, add the path to the clang binaries to `PATH` via (ideally, put it into `.bashrc`/`.zshrc`):
​
    ```
    export PATH=$HOME/ThirdParty/bin:$PATH
    ```
​
- Build UE4. To that end, we will need more CPU cores. Let's exit the current session and create a 32 xeon cores slurm session:
​
    ```
    exit
    srun -p cpu  -c 32 --qos=inter --pty bash
    cd UnrealEngine/UnrealEngine-4.26.2-release
    ./Setup.sh
    ./GenerateProjectFiles.sh
    make
    ```
​
### InteriorSim
​
- Install InteriorSim (following the tutorial in the main readme file)
- On Slurm, ensure to have at least one GPU using e.g.,
​
    ```
    srun -p g24 --gres=gpu:1 -c 14  --qos=inter --pty bash
    ```
​
    otherwise the following message will appear: `LogLinux: Warning: MessageBox: Cannot find a compatible Vulkan driver (ICD). Please look at the Getting Started guide for additional information.: Incompatible Vulkan driver found!: Cursors are not currently supported.`
​
- Create symbolic links to the different .pak files stored in `/export/share/projects/InteriorSim/scene_manager/download_data/v4/` to be able to load each environment (replace `<PATH_INTERIOR_SIM>` with real path):
​
    ```
    cd /export/share/projects/InteriorSim/scene_manager/download_data/v4
    for f in *; do ln -s "/export/share/projects/InteriorSim/scene_manager/download_data/v4/$f/paks/Linux/$f/${f}_Linux.pak" "<PATH_INTERIOR_SIM>/interiorsim/code/unreal_projects/RobotProject/dist/LinuxNoEditor/RobotProject/Content/Paks/${f}_Linux.pak"; echo "/export/share/projects/InteriorSim/scene_manager_download_data/v4/$f/paks/Linux/$f/${f}_Linux.pak"; done
    ```
​
- Check that your links are valid:
​
    ```
    cd $HOME/UnrealEngine/interiorsim/code/unreal_projects/RobotProject/dist/LinuxNoEditor/RobotProject/Content/Paks
    ls -las
    ```
​
You should now be able to run the OpenBot agent in any environment. Have Fun !
