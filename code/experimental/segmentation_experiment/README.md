# iclr_experiment

This folder contains reference training scripts for semantic segmentation.
This is intended to be used for fast track ICLR interiorsim paper submission.

## Cluster session setup

- Create and activate your pytorch environment:

`conda create -n pytorch-env pip python=3.9`

`conda activate pytorch-env`

- Install cudatoolkit version 11.3.1:

`conda install -c anaconda cudatoolkit`

- Install CUDNN version 8.2.1 (compatible with cudatoolkit 11.3.1)

`conda install -c anaconda cudnn`

- Install the latest stable pytorch version (currently 1.12.1) using the nice command generation tool from https://pytorch.org/

`conda install pytorch torchvision -c pytorch`

- Getting ready to run the segmentation training code

`pip install pycocotools h5py scipy tensorboard`
## Dataset setup

We here consider the dataset `mydataset` and assume that it contains two folders, named `rgb` and `sem_seg` containing the images and masks respectively.  
Create a `split` directory that will contain two textfiles, providing the references to the images/masks of the training set and images/masks of the test set: 

`cd mydataset`

`mkdir split`

`touch split/train.txt`

Fill the train reference file with the names of the desired images:

`cd mydataset/rgb`

`ls > ../split/train.txt` 

Here we assume that the images and masks have the same names and extentions... 


## Training commands (slurm API on the cluster)

We here assume that you already have a properly set `pytorch-env` virtual environment. Move to the `iclr_experiment/slurm` folder:

`cd iclr_experiment/slurm`

Open the desired script and set the `anaconda3` path to match your settings in the `source` command. If `anaconda3` is in your home folder, you should get something like:

`source $HOME/anaconda3/bin/activate pytorch-env`

Then launch the desired training by executing:

`sbatch run_training_<dataset>.sh`

where `<dataset>` is set to match the name of your dataset (i.e. `interiorsim`, `ai2thor`, `replica`, `replica_cad`, `mp3d`, `hm3d`, `tdw_filtered` and `nyuv2`)

You can check the state of the training process by looking at the content of the corresponding `slurm-XXXX.out` file, where `XXXX` is the pid of your script (displayed when you execute the `sbatch` comand).

## Training commands (interactive session for debug purposes)

We here assume that you already have a properly set `pytorch-env` virtual environment. First of all, make sure that you are on a slurm interactive sesison with the required number of allocated GPUs. 
For instance, to create an interactive session with 8 GPUs and 112 cpu, type:

`srun -p g24 --gres=gpu:8 -c 112 --qos=inter --pty bash`

Then activate your `pytorch-env` virtual environment:

`conda activate pytorch-env`

Execute one of the following commands to run the training script in an interactive session, while adequately modifying the following flags:

`--data-path=/path/to/dataset`

`--nproc_per_node=<number_of_gpus_available>`

`--output_dir=/path/to/output/directory` 

This flag denotes the location of the generated model. AS the resulting files can be cumbersome, they should either be sent in your `$WORK` partition or in the shared InteriorSim partition.

`--dataset <name_of_the_simulator>` 

So far, `<name_of_the_simulator>` can be set to `interiorsim`, `ai2thor`, `replica`, `replica_cad`, `mp3d`, `hm3d`, `tdw_filtered` and `nyuv2`

### fcn_resnet50
```
torchrun --nproc_per_node=8 train.py --data-path=/export/share/projects/InteriorSim/iclr_dataset --lr 0.02 --dataset interiorsim --output_dir=/export/share/projects/InteriorSim/iclr_dataset/interiorsim/output -b 4 --model fcn_resnet50 --aux-loss --weights-backbone ResNet50_Weights.IMAGENET1K_V1
```

### fcn_resnet101
```
torchrun --nproc_per_node=8 train.py --data-path=/export/share/projects/InteriorSim/iclr_dataset --lr 0.02 --dataset interiorsim --output_dir=/export/share/projects/InteriorSim/iclr_dataset/interiorsim/output -b 4 --model fcn_resnet101 --aux-loss --weights-backbone ResNet101_Weights.IMAGENET1K_V1
```

### deeplabv3_resnet50
```
torchrun --nproc_per_node=8 train.py --data-path=/export/share/projects/InteriorSim/iclr_dataset --lr 0.02 --dataset interiorsim --output_dir=/export/share/projects/InteriorSim/iclr_dataset/interiorsim/output -b 4 --model deeplabv3_resnet50 --aux-loss --weights-backbone ResNet50_Weights.IMAGENET1K_V1
```

### deeplabv3_resnet101
```
torchrun --nproc_per_node=8 train.py --data-path=/export/share/projects/InteriorSim/iclr_dataset --lr 0.02 --dataset interiorsim --output_dir=/export/share/projects/InteriorSim/iclr_dataset/interiorsim/output -b 4 --model deeplabv3_resnet101 --aux-loss --weights-backbone ResNet101_Weights.IMAGENET1K_V1
```

### deeplabv3_mobilenet_v3_large
```
torchrun --nproc_per_node=8 train.py --data-path=/export/share/projects/InteriorSim/iclr_dataset --dataset interiorsim --output_dir=/export/share/projects/InteriorSim/iclr_dataset/interiorsim/output -b 4 --model deeplabv3_mobilenet_v3_large --aux-loss --wd 0.000001 --weights-backbone MobileNet_V3_Large_Weights.IMAGENET1K_V1
```

### lraspp_mobilenet_v3_large
```
torchrun --nproc_per_node=8 train.py --data-path=/export/share/projects/InteriorSim/iclr_dataset --dataset interiorsim --output_dir=/export/share/projects/InteriorSim/iclr_dataset/interiorsim/output -b 4 --model lraspp_mobilenet_v3_large --wd 0.000001 --weights-backbone MobileNet_V3_Large_Weights.IMAGENET1K_V1
```

