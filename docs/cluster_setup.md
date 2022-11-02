# Cluster Setup

"Cluster-HowTo" to rapidly get through the jungle of cuda and learning framework versions, thereby avoiding time-consuming incompatibility issues.

## First steps

- Start by logging into the cluster:
    ```console
    ssh user@isl-iam1.rr.intel.com
    ```
- You should find yourself in one of the two login nodes of the cluster. The **golden rule** here is to avoid doing anything that could potentially slow down other users of the cluster, i.e. no downloading, no computing etc. The login nodes should _only_ be used to start compute sessions through the `srun` or `sbatch` slurm commands. 
- You are **strongly** advised to check the cluster documentation on the [wiki](https://wiki.ith.intel.com/display/VCL/Slurm+Cluster).
- Every user of the cluster has read-write access to 2 distinct partitions:
  - `$HOME` is a fast 300Gb space that should be used for installing software (e.g. Unreal, Conda, ...)
  - `$WORK` is a 1Tb space for storing data
- The SPEAR project also comes with a dedicated shared storage space located in `/export/share/projects/InteriorSim/`, containing in particular the packaged `.pak` files of every environment within the subfolder `/export/share/projects/InteriorSim/scene_manager_download_data/v4.`
- The classic execution scheme of a process on the cluster first consists of allocating computational resources -- such as CPUs or GPUs -- to be used by this process. Computations can then either be executed in the background (classic session) or in a dedicated terminal interface (interative session). 
- The setup of your environment must be carried out in a dedicated cluster _interactive session_ (i.e. with feedback in a terminal). To do so, you may use  the following command (with the flags `--qos=inter --pty bash` to specify that your session should be interactive):
    ```console
    srun -p cpu -c 2 --qos=inter --pty bash
    ```
- Be aware that although interactive sessions have the highest priority on the cluster, they only have a 2h lifetime. They are therefore ideal for building or debugging purposes but should typically not be used for large-scale computations that may often be distributed over several days. In the previous command, a set of 2 Xeon CPUs are allocated to the interactive session (using the flag `-c 2`) which is unually enough for most download/install/monitoring processes, but can of course be adjusted depending on your needs.  

## Anaconda3

- Once logged into your slurm interactive session, the first step is to install anaconda, which will provide a powerful interface for managing virtual python environments and allow you to install the required drivers and learning frameworks you will later need in your applications:
    ```console
    wget https://repo.anaconda.com/archive/Anaconda3-2022.05-Linux-x86_64.sh
    sh Anaconda3-2022.05-Linux-x86_64.sh
    ```
- *Optionally*, update anaconda:
  - update the conda package manager to the latest version (may have to `source $HOME/.bashrc` first)
    ```console
    conda update conda
    ```
  - use conda to update Anaconda to the latest version
    ```console
    conda update anaconda
    ```
## Pytorch
The following shows how to set a pytorch environment, with access to GPU computing power:
- Use either an existing conda environment or create and activate your pytorch environment:
    ```console
    conda create -n pytorch-env pip python=3.9
    conda activate pytorch-env
    ```
- Install cudatoolkit version 11.3.1:
    ```console
    conda install -c anaconda cudatoolkit
    ```
- Install CUDNN version 8.2.1 (compatible with cudatoolkit 11.3.1)
    ```console
    conda install -c anaconda cudnn
    ```
- Install the latest stable pytorch version (currently 1.12.1) using the nice command generation tool from <https://pytorch.org/>
    ```console
    conda install pytorch torchvision -c pytorch
    ```
## Tensorflow
The following shows how to set a tensorflow environment, with access to GPU computing power:
- Use either an existing conda environment or create and activate your tensorflow environment:
    ```console
    conda create -n tensorflow-env pip python=3.9
    conda activate tensorflow-env
    ```
- Install cudatoolkit version 11.3.1
    ```console
    conda install -c anaconda cudatoolkit
    ```
- Install CUDNN version 8.2.1 (compatible with cudatoolkit 11.3.1)
    ```console
    conda install -c anaconda cudnn
    ```
- Install tensorflow 2.5.2 using the pip
    ```console
    pip install tensorflow==2.5.2
    ```
## Unreal Engine
Let's now take a look on how to build the Unreal Engine:
- I here assume that you already took the Required Setup (c.f. Step 1 in [Unreal Engine Documentation](https://docs.unrealengine.com/4.27/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/)) to get access to the UE4 git repo
- **Don't clone this repo**: it's HUGE. Download the 4.26 release instead. As follows
  - Access to the private GitHub repository needs to be requested by following Step 1 in [Unreal Engine Documentation](https://docs.unrealengine.com/4.27/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/), and by joining the EpicGames organisation (clicking on the email sent by them).
  - Once access to the UnrealEngine repository is granted, the link "https://github.com/EpicGames/UnrealEngine/archive/refs/tags/4.26.2-release.zip" should be accessible. To be able to curl/wget the link, a [personal access token](https://github.com/settings/tokens) needs to be generated on GitHub.
  - Then, the zip file can be downloaded by replacing `<personal_access_token>` with the personal access token generated from GitHub.
    ```console
    cd $HOME
    mkdir UnrealEngine
    cd UnrealEngine
    curl -H "Authorization: token <personal_access_token>" -L https://github.com/EpicGames/UnrealEngine/archive/refs/tags/4.26.2-release.zip > 4.26.2-release.zip
    unzip 4.26.2-release.zip
    ```
- Installing cmake (sudo access is not provided on cluster): Follow instructions from [this link](https://pachterlab.github.io/kallisto/local_build.html). Steps:
  - Requirements: C++11 compatible compiler (>= g++-4.8), zlib, make
  - **Important**: It is assumed that all ThirdParty libraries are installed to `$HOME/ThirdParty`. The prefix argument will point to there and specifies where the binary files are located, ensure that it aligns with `PATH`/`LD_LIBRARY_PATH`.
  - We need to extend the paths (ideally, put this into `.bashrc`/`.zshrc`):
    ```console
    export PATH=$HOME/ThirdParty/bin:$PATH
    export LD_LIBRARY_PATH=$HOME/ThirdParty/lib/:$LD_LIBRARY_PATH
    ```
  - download Cmake from [website](https://cmake.org/download/)
  - Install Cmake to the folder `$HOME/ThirdParty` (or any other path that was specified in prefix and where `PATH` includes it):
    ```console
    tar -xf cmake*.tar.gz
    cd cmake*
    ./configure --prefix=$HOME/ThirdParty
    make
    make install
    ```
  - Ensure correct installation by running
    ```console
    cmake --version
    ```
- It is also necessary to download `clang 11.0.0` to build UE4 on the cluster.
  - The most convenient is to download the prebuilt binaries:
    ```console
    cd $HOME
    mkdir -p ThirdParty/clang
    cd ThirdParty/clang
    wget https://github.com/llvm/llvm-project/releases/download/llvmorg-11.0.0/clang+llvm-11.0.0-x86_64-linux-gnu-ubuntu-20.04.tar.xz
    tar -xzvf clang+llvm-11.0.0-x86_64-linux-gnu-ubuntu-20.04.tar.xz
    mv clang+llvm-11.0.0-x86_64-linux-gnu-ubuntu-20.04/* $HOME/ThirdParty/
    ```
  - Note: if not done in the `cmake` step, add the path to the clang binaries to `PATH` via (ideally, put it into `.bashrc`/`.zshrc`):
    ```console
    export PATH=$HOME/ThirdParty/bin:$PATH
    ```
- We should by now be able to build UE4. To that end, we will need more CPU cores. Let's exit the current session and create a 32 Xeon cores slurm session:
    ```console
    exit
    srun -p cpu  -c 32 --qos=inter --pty bash
    ```
- Let's finally run Unreal's main build scripts:
    ```console
    cd UnrealEngine/UnrealEngine-4.26.2-release
    ./Setup.sh
    ./GenerateProjectFiles.sh
    make
    ```
    
## SPEAR
- Install SPEAR following the tutorial in the main [ReadMe](https://github.com/isl-org/interiorsim/blob/main/docs/getting_started.md) file.
- On Slurm, ensure to have at least one GPU using e.g.,
    ```console
    srun -p g24 --gres=gpu:1 -c 14  --qos=inter --pty bash
    ```
    otherwise the following message will appear: `LogLinux: Warning: MessageBox: Cannot find a compatible Vulkan driver (ICD). Please look at the Getting Started guide for additional information.: Incompatible Vulkan driver found!: Cursors are not currently supported.`
- Create symbolic links to the different .pak files stored in `/export/share/projects/InteriorSim/scene_manager/download_data/v4/` to be able to load each environment (replace `<PATH_INTERIOR_SIM>` with real path):
    ```console
    cd /export/share/projects/InteriorSim/scene_manager/download_data/v4
    for f in *; do ln -s "/export/share/projects/InteriorSim/scene_manager/download_data/v4/$f/paks/Linux/$f/${f}_Linux.pak" "<PATH_INTERIOR_SIM>/interiorsim/code/unreal_projects/RobotProject/dist/LinuxNoEditor/RobotProject/Content/Paks/${f}_Linux.pak"; echo "/export/share/projects/InteriorSim/scene_manager_download_data/v4/$f/paks/Linux/$f/${f}_Linux.pak"; done
    ```
- Check that your links are valid:
    ```console
    cd $HOME/UnrealEngine/interiorsim/code/unreal_projects/RobotProject/dist/LinuxNoEditor/RobotProject/Content/Paks
    ls -las
    ```
You should now be able to run the OpenBot agent in any environment. Have Fun !
