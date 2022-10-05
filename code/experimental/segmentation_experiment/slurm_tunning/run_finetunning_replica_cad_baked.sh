#!/bin/bash

#SBATCH -p gpu
#SBATCH --gres=gpu:1
#SBATCH -c 14
#SBATCH -w isl-gpu13

cd ../segmentation
source $HOME/anaconda3/bin/activate pytorch-env

torchrun --nproc_per_node=1 train.py --epochs 200 --data-path=/export/share/datasets/nyu_depth_v2/labeled --lr 0.02 --dataset nyuv2 --resume=/export/share/projects/InteriorSim/iclr_results/replica/output_final/model_99.pth --output-dir=/export/share/projects/InteriorSim/iclr_results/replica_cad_baked/output_final_tunning -b 4 --model deeplabv3_resnet101 --aux-loss --weights-backbone ResNet101_Weights.IMAGENET1K_V2
