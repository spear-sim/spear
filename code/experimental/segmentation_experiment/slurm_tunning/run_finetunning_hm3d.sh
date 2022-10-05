#!/bin/bash

#SBATCH -p g24
#SBATCH --gres=gpu:1
#SBATCH -c 10
#SBATCH -w isl-gpu4

cd ../segmentation
source $HOME/anaconda3/bin/activate pytorch-env

torchrun --nproc_per_node=1 train.py --epochs 200 --data-path=/export/share/datasets/nyu_depth_v2/labeled --lr 0.02 --dataset nyuv2 --resume=/export/share/projects/InteriorSim/iclr_results/hm3d/output_final/model_99.pth --output-dir=/export/share/projects/InteriorSim/iclr_results/hm3d/output_final_tunning2 -b 4 --model deeplabv3_resnet101 --aux-loss
