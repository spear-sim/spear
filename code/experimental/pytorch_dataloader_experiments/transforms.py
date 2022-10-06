import os

import numpy as np
import scipy.io
import torch
import torchvision

# The NYU13 classes are as follows:
# [None, "bed", "books", "ceiling", "chair", "floor", "furniture", "objects", "painting", "sofa", "table", "tv", "wall", "window"]
#
# Note that some references refer to the "painting" class as a "picture" class [1], but others refer to it
# as "painting" [2]. The mapping from NYU40 classes to NYU13 classes was obtained from [3].
#
# [1] https://cs.nyu.edu/~deigen/deigen-thesis.pdf
# [2] https://openaccess.thecvf.com/content_ICCV_2017/papers/McCormac_SceneNet_RGB-D_Can_ICCV_2017_paper.pdf
# [3] https://github.com/ankurhanda/nyuv2-meta-data/raw/master/class13Mapping.mat
#

NYU40_SEMANTIC_ID_TO_NYU13_SEMANTIC_ID = \
    np.array([0, 12, 5, 6, 1, 4, 9, 10, 12, 13, 6, 8, 6, 13, 10, 6, 13, 6, 7, 7, 5, 7, 3, 2, 6, 11, 7, 7, 7, 7, 7, 7, 6, 7, 7, 7, 7, 7, 7, 6, 7])

class AllModalitiesTransform:

    def __init__(self, img_transform):
        self.img_transform = img_transform
 
    def __call__(self, item):
        for k in item.keys():
            item[k] = self.img_transform(item[k])
        return item

class SingleModalityTransform:

    def __init__(self, modality, img_transform):
        self.modality = modality
        self.img_transform = img_transform

    def __call__(self, item):
        item[self.modality] = self.img_transform(item[self.modality])
        return item

class NYU894ToNYU40:

    def __init__(self, data_dir):
        nyu40_label_mapping_mat_file                 = os.path.join(data_dir, "classMapping40.mat")
        nyu40_label_mapping_mat                      = scipy.io.loadmat(nyu40_label_mapping_mat_file)
        nyu894_semantic_id_to_nyu40_semantic_id_     = nyu40_label_mapping_mat["mapClass"][0][:]
        self.nyu894_semantic_id_to_nyu40_semantic_id = np.r_[ 0, nyu894_semantic_id_to_nyu40_semantic_id_ ]

    def __call__(self, img):
        mask = img != -1
        img[mask] = self.nyu894_semantic_id_to_nyu40_semantic_id[img[mask]]
        return img

class NYU40ToNYU13:

    def __call__(self, img):
        mask = img != -1
        img[mask] = NYU40_SEMANTIC_ID_TO_NYU13_SEMANTIC_ID[img[mask]]
        return img

class MapValues:

    def __init__(self, map):
        self.map = map

    def __call__(self, img):
        for k in self.map:
            img[img == k] = self.map[k]
        return img

class AsType:

    def __init__(self, dtype):
        self.dtype = dtype

    def __call__(self, img):
        return img.astype(self.dtype)

class CastLong:
    def __call__(self, img):
        return img.long()

#####################
# Data Augmentations
######################

class HorizontalFlip:
    def __init__(self, p=0.5):
        self.p = p

    def __call__(self, item):
        if torch.rand(1) < self.p:
            for k in item.keys():
                item[k] = torchvision.transforms.functional.hflip(item[k])
        return item

class VerticalFlip:
    def __init__(self, p=0.5):
        self.p = p

    def __call__(self, item):
        if torch.rand(1) < self.p:
            for k in item.keys():
                item[k] = torchvision.transforms.functional.vflip(item[k])
        return item

class ColorJitter:
    def __init__(self, brightness=0, contrast=0, saturation=0, hue=0, p=0.5):
        self.jitter = torchvision.transforms.ColorJitter(
            brightness, contrast, saturation, hue)
        self.p = p

    def __call__(self, img):
        if torch.rand(1) < self.p:
            img = self.jitter(img)
        return img

class RandomResizedCrop:
    """Wrapper around torchvision.transforms.RandomResizedCrop for multiple modalities."""
    def __init__(
        self,
        size,
        scale=(0.08, 1.0),
        ratio=(3. / 4., 4. / 3.),
        interpolation_map=None
    ):
        self.size = size
        self.scale = scale
        self.ratio = ratio
        self.interpolation_map = interpolation_map

    def __call__(self, item): 
        key = list(item.keys())[0]
        # Sample the random crop just once
        i, j, h, w = torchvision.transforms.RandomResizedCrop.get_params(item[key], self.scale, self.ratio)
        for k in item.keys():
            if self.interpolation_map is None or k not in self.interpolation_map:
                interpolation = torchvision.transforms.InterpolationMode.BILINEAR
            else:
                interpolation = self.interpolation_map[k]
            item[k] = torchvision.transforms.functional.resized_crop(item[k], i, j, h, w, self.size, interpolation)
        return item

class Resize:
    def __init__(
        self,
        size,
        interpolation=torchvision.transforms.InterpolationMode.BILINEAR
    ):
        self.resize = torchvision.transforms.Resize(size, interpolation=interpolation)

    def __call__(self, img):
        return self.resize(img)
