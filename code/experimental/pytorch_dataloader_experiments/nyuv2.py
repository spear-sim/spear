import h5py
import numpy as np
import os
import scipy.io
import torch

ALL_MODALITIES = [
    "color",
    "depth",
    "raw_depth",
    "semantic",
    "semantic_instance" ]

class NYUv2Dataset(torch.utils.data.Dataset):

    def __init__(
        self,
        data_dir,
        split,
        modalities="all",
        transform=None):

        assert split in ["train", "test"]
        assert modalities == "all" or set(modalities) <= set(ALL_MODALITIES)

        nyu_depth_v2_labeled_mat_file = os.path.join(data_dir, "nyu_depth_v2_labeled.mat")
        nyu_depth_v2_splits_mat_file  = os.path.join(data_dir, "splits.mat")

        with h5py.File(nyu_depth_v2_labeled_mat_file, "r") as f:
                nyu_depth_v2_images     = f["images"][:]
                nyu_depth_v2_depths     = f["depths"][:]
                nyu_depth_v2_raw_depths = f["rawDepths"][:]
                nyu_depth_v2_instances  = f["instances"][:]
                nyu_depth_v2_labels     = f["labels"][:]

        nyu_depth_v2_splits_mat = scipy.io.loadmat(nyu_depth_v2_splits_mat_file)
        
        if split == "train":
            nyu_depth_v2_split = nyu_depth_v2_splits_mat["trainNdxs"].ravel() - 1
        else:
            nyu_depth_v2_split = nyu_depth_v2_splits_mat["testNdxs"].ravel() - 1

        self.nyu_depth_v2_images     = nyu_depth_v2_images[nyu_depth_v2_split]
        self.nyu_depth_v2_depths     = nyu_depth_v2_depths[nyu_depth_v2_split]
        self.nyu_depth_v2_raw_depths = nyu_depth_v2_raw_depths[nyu_depth_v2_split]
        self.nyu_depth_v2_instances  = nyu_depth_v2_instances[nyu_depth_v2_split]
        self.nyu_depth_v2_labels     = nyu_depth_v2_labels[nyu_depth_v2_split]
        
        if modalities == "all":
            self.modalities = ALL_MODALITIES
        else:
            self.modalities = modalities

        self.transform = transform

    def __getitem__(self, index):

        item = {}
        
        for m in self.modalities:
            if m == "color":
                item[m] = np.array(self.nyu_depth_v2_images[index].T)
            elif m == "depth":
                item[m] = np.array(self.nyu_depth_v2_depths[index].T)
            elif m == "raw_depth":
                item[m] = np.array(self.nyu_depth_v2_raw_depths[index].T)
            elif m == "semantic":
                item[m] = np.array(self.nyu_depth_v2_labels[index].T)
            elif m == "semantic_instance":
                item[m] = np.array(self.nyu_depth_v2_instances[index].T)
            else:
                assert False

        if self.transform is not None:
            item = self.transform(item)

        return item

    def __len__(self):
        return self.nyu_depth_v2_images.shape[0]
