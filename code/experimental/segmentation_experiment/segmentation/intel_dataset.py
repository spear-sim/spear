import os
import h5py
import numpy as np
import scipy.io
from matplotlib import pyplot as plt
from typing import Any, Callable, Optional, Tuple, List
from PIL import Image
import csv
from torchvision.datasets.vision import VisionDataset
from torchvision.datasets.utils import verify_str_arg


class _ImageSegmentationBase(VisionDataset):
    _SIM_DIR: str

    def __init__(
        self,
        root: str,
        split: str = "train",
        transform: Optional[Callable] = None,
        target_transform: Optional[Callable] = None,
        transforms: Optional[Callable] = None,
    ):
        super().__init__(root, transforms, transform, target_transform)

        self.debug = True

        assert split in ["train", "test"]
        self.split = split

        # Check that the dataset folder exists.
        dataset_root = os.path.join(self.root, self._SIM_DIR)
        if not os.path.isdir(dataset_root):
            raise RuntimeError("Dataset not found or corrupted.")

        # Read the .txt file containing the references of the images to be used for training purpose and for testing purpose, respectively.
        splits_dir = os.path.join(dataset_root, "split")
        split_file = os.path.join(splits_dir, self.split.rstrip("\n") + ".txt")
        with open(os.path.join(split_file)) as f:
            file_names = [x.strip() for x in f.readlines()]

        # Store the name of the training/testing images.
        image_dir = os.path.join(dataset_root, "rgb")
        self.images = [os.path.join(image_dir, x) for x in file_names]

        # Store the name of the training/testing target masks.
        target_dir = os.path.join(dataset_root, "sem_seg")
        self.targets = [os.path.join(target_dir, x) for x in file_names]

        assert len(self.images) == len(self.targets)
        assert len(self.images) > 0

        print(f"Loaded {len(self.images)} images for the {self._SIM_DIR} {self.split} dataset.")

        self.palette = np.zeros(2000)
        with open(self._SIM_DIR + ".csv") as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if line_count == 0:
                    line_count += 1
                else:
                    self.palette[line_count - 1] = row[0]
                    line_count += 1
            print(f'Remapped {line_count} categories to {self.palette.max() + 1} categories.')
        

    def __len__(self) -> int:
        return len(self.images)

    @property
    def masks(self) -> List[str]:
        return self.targets

    def __getitem__(self, index: int) -> Tuple[Any, Any]:
        """
        Args:
            index (int): Index

        Returns:
            tuple: (image, target) where target is the image segmentation.
        """
        img = Image.open(self.images[index]).convert('RGB')
        mask = Image.open(self.masks[index])
        target = Image.fromarray(np.uint8(self.palette[np.clip(mask, 0, 1999)]), mode='P').resize(img.size)

        # if self.debug == True:

        #     fig = plt.figure(figsize=(7, 7))
        #     rows = 2
        #     columns = 2
        #     fig.add_subplot(rows, columns, 1)
        #     plt.imshow(img)
        #     plt.axis('off')
        #     plt.title("Raw RGB image")
        #     fig.add_subplot(rows, columns, 2)
        #     plt.imshow(mask)
        #     plt.axis('off')
        #     plt.title("Raw mask")

        #     if self.transforms is not None:
        #         img, target = self.transforms(img, target)

        #     fig.add_subplot(rows, columns, 3)
        #     plt.imshow(img.permute(1, 2, 0))
        #     plt.axis('off')
        #     plt.title("Transformed RGB image")
        #     fig.add_subplot(rows, columns, 4)
        #     plt.imshow(target)
        #     plt.axis('off')
        #     plt.title("Transformed mask")
        #     plt.show()
        
        # else:

        if self.transforms is not None:
            img, target = self.transforms(img, target)

        return img, target


class InteriorSimSegmentation(_ImageSegmentationBase):
    """ InteriorSim Segmentation Dataset.

    Args:
        root (string): Root directory of the InteriorSim Dataset.
        split (string, optional): Select the split to use, ``"train"`` or ``"val"``. 
        transform (callable, optional): A function/transform that  takes in an PIL image
            and returns a transformed version. E.g, ``transforms.RandomCrop``
        target_transform (callable, optional): A function/transform that takes in the
            target and transforms it.
        transforms (callable, optional): A function/transform that takes input sample and its target as entry
            and returns a transformed version.
    """

    _SIM_DIR = "interiorsim"

    def __init__(
        self,
        root: str,
        split: str = "train",
        transform: Optional[Callable] = None,
        target_transform: Optional[Callable] = None,
        transforms: Optional[Callable] = None,
    ):
        super().__init__(root, split, transform, target_transform, transforms)


class Ai2ThorSegmentation(_ImageSegmentationBase):
    """ Ai2Thor Segmentation Dataset.

    Args:
        root (string): Root directory of the Ai2Thor Dataset.
        split (string, optional): Select the split to use, ``"train"`` or ``"val"``. 
        transform (callable, optional): A function/transform that  takes in an PIL image
            and returns a transformed version. E.g, ``transforms.RandomCrop``
        target_transform (callable, optional): A function/transform that takes in the
            target and transforms it.
        transforms (callable, optional): A function/transform that takes input sample and its target as entry
            and returns a transformed version.
    """

    _SIM_DIR = "ai2thor"

    def __init__(
        self,
        root: str,
        split: str = "train",
        transform: Optional[Callable] = None,
        target_transform: Optional[Callable] = None,
        transforms: Optional[Callable] = None,
    ):
        super().__init__(root, split, transform, target_transform, transforms)


class TdwSegmentation(_ImageSegmentationBase):
    """ Replica Segmentation Dataset.

    Args:
        root (string): Root directory of the TDW Dataset.
        split (string, optional): Select the split to use, ``"train"`` or ``"val"``. 
        transform (callable, optional): A function/transform that  takes in an PIL image
            and returns a transformed version. E.g, ``transforms.RandomCrop``
        target_transform (callable, optional): A function/transform that takes in the
            target and transforms it.
        transforms (callable, optional): A function/transform that takes input sample and its target as entry
            and returns a transformed version.
    """

    _SIM_DIR = "tdw_filtered"

    def __init__(
        self,
        root: str,
        split: str = "train",
        transform: Optional[Callable] = None,
        target_transform: Optional[Callable] = None,
        transforms: Optional[Callable] = None,
    ):
        super().__init__(root, split, transform, target_transform, transforms)


class ReplicaSegmentation(_ImageSegmentationBase):
    """ Replica Segmentation Dataset.

    Args:
        root (string): Root directory of the Replica Dataset.
        split (string, optional): Select the split to use, ``"train"`` or ``"val"``. 
        transform (callable, optional): A function/transform that  takes in an PIL image
            and returns a transformed version. E.g, ``transforms.RandomCrop``
        target_transform (callable, optional): A function/transform that takes in the
            target and transforms it.
        transforms (callable, optional): A function/transform that takes input sample and its target as entry
            and returns a transformed version.
    """

    _SIM_DIR = "replica"

    def __init__(
        self,
        root: str,
        split: str = "train",
        transform: Optional[Callable] = None,
        target_transform: Optional[Callable] = None,
        transforms: Optional[Callable] = None,
    ):
        super().__init__(root, split, transform, target_transform, transforms)


class ReplicaCadSegmentation(_ImageSegmentationBase):
    """ Replica Segmentation Dataset.

    Args:
        root (string): Root directory of the Replica CAD Dataset.
        split (string, optional): Select the split to use, ``"train"`` or ``"val"``. 
        transform (callable, optional): A function/transform that  takes in an PIL image
            and returns a transformed version. E.g, ``transforms.RandomCrop``
        target_transform (callable, optional): A function/transform that takes in the
            target and transforms it.
        transforms (callable, optional): A function/transform that takes input sample and its target as entry
            and returns a transformed version.
    """

    _SIM_DIR = "replica_cad"

    def __init__(
        self,
        root: str,
        split: str = "train",
        transform: Optional[Callable] = None,
        target_transform: Optional[Callable] = None,
        transforms: Optional[Callable] = None,
    ):
        super().__init__(root, split, transform, target_transform, transforms)

class ReplicaCadBakedSegmentation(_ImageSegmentationBase):
    """ Replica Segmentation Dataset.

    Args:
        root (string): Root directory of the Replica CAD Dataset.
        split (string, optional): Select the split to use, ``"train"`` or ``"val"``. 
        transform (callable, optional): A function/transform that  takes in an PIL image
            and returns a transformed version. E.g, ``transforms.RandomCrop``
        target_transform (callable, optional): A function/transform that takes in the
            target and transforms it.
        transforms (callable, optional): A function/transform that takes input sample and its target as entry
            and returns a transformed version.
    """

    _SIM_DIR = "replica_cad_baked"

    def __init__(
        self,
        root: str,
        split: str = "train",
        transform: Optional[Callable] = None,
        target_transform: Optional[Callable] = None,
        transforms: Optional[Callable] = None,
    ):
        super().__init__(root, split, transform, target_transform, transforms)



class Mp3DSegmentation(_ImageSegmentationBase):
    """ Mp3D Segmentation Dataset.

    Args:
        root (string): Root directory of the Mp3D Dataset.
        split (string, optional): Select the split to use, ``"train"`` or ``"val"``. 
        transform (callable, optional): A function/transform that  takes in an PIL image
            and returns a transformed version. E.g, ``transforms.RandomCrop``
        target_transform (callable, optional): A function/transform that takes in the
            target and transforms it.
        transforms (callable, optional): A function/transform that takes input sample and its target as entry
            and returns a transformed version.
    """

    _SIM_DIR = "mp3d"

    def __init__(
        self,
        root: str,
        split: str = "train",
        transform: Optional[Callable] = None,
        target_transform: Optional[Callable] = None,
        transforms: Optional[Callable] = None,
    ):
        super().__init__(root, split, transform, target_transform, transforms)


class Hm3DSegmentation(_ImageSegmentationBase):
    """ Hm3D Segmentation Dataset.

    Args:
        root (string): Root directory of the Hm3D Dataset.
        split (string, optional): Select the split to use, ``"train"`` or ``"val"``. 
        transform (callable, optional): A function/transform that  takes in an PIL image
            and returns a transformed version. E.g, ``transforms.RandomCrop``
        target_transform (callable, optional): A function/transform that takes in the
            target and transforms it.
        transforms (callable, optional): A function/transform that takes input sample and its target as entry
            and returns a transformed version.
    """

    _SIM_DIR = "hm3d"

    def __init__(
        self,
        root: str,
        split: str = "train",
        transform: Optional[Callable] = None,
        target_transform: Optional[Callable] = None,
        transforms: Optional[Callable] = None,
    ):
        super().__init__(root, split, transform, target_transform, transforms)


ALL_MODALITIES = [
    "color",
    "depth",
    "raw_depth",
    "semantic",
    "semantic_instance"]


class NYUv2Segmentation(VisionDataset):
    """ NYUv2 Segmentation Dataset.

    Args:
        root (string): Root directory of the NYUv2 Dataset.
        split (string, optional): Select the split to use, ``"train"`` or ``"val"``. 
        transform (callable, optional): A function/transform that  takes in an PIL image
            and returns a transformed version. E.g, ``transforms.RandomCrop``
        target_transform (callable, optional): A function/transform that takes in the
            target and transforms it.
        transforms (callable, optional): A function/transform that takes input sample and its target as entry
            and returns a transformed version.
    """

    def __init__(
        self,
        root: str,
        split: str = "train",
        modalities: str = "all",
        transform: Optional[Callable] = None,
        target_transform: Optional[Callable] = None,
        transforms: Optional[Callable] = None,
    ):
        super().__init__(root, transforms, transform, target_transform)

        self.debug = True

        assert split in ["train", "test"]
        self.split = split

        self.palette = np.zeros(2000)
        with open("nyuv2.csv") as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if line_count == 0:
                    line_count += 1
                else:
                    self.palette[line_count - 1] = row[0]
                    line_count += 1
            print(f'Remapped {line_count} categories to {self.palette.max() + 1} categories.')

        assert modalities == "all" or set(modalities) <= set(ALL_MODALITIES)

        nyu_depth_v2_labeled_mat_file = os.path.join(root, "nyu_depth_v2_labeled.mat")
        nyu_depth_v2_splits_mat_file = os.path.join(root, "splits.mat")

        with h5py.File(nyu_depth_v2_labeled_mat_file, "r") as f:
            nyu_depth_v2_images = f["images"][:]
            #nyu_depth_v2_depths = f["depths"][:]
            #nyu_depth_v2_raw_depths = f["rawDepths"][:]
            #nyu_depth_v2_instances = f["instances"][:]
            nyu_depth_v2_labels = f["labels"][:]

        print(f'Loaded {len(nyu_depth_v2_images)} images.')        

        nyu_depth_v2_splits_mat = scipy.io.loadmat(nyu_depth_v2_splits_mat_file)

        if split == "train":
            nyu_depth_v2_split = nyu_depth_v2_splits_mat["trainNdxs"].ravel() - 1
            print(f'Training set: {len(nyu_depth_v2_split)} images.')    
        else:
            nyu_depth_v2_split = nyu_depth_v2_splits_mat["testNdxs"].ravel() - 1
            print(f'Test set: {len(nyu_depth_v2_split)} images.')  

        self.nyu_depth_v2_images = nyu_depth_v2_images[nyu_depth_v2_split]
        #self.nyu_depth_v2_depths = nyu_depth_v2_depths[nyu_depth_v2_split]
        #self.nyu_depth_v2_raw_depths = nyu_depth_v2_raw_depths[nyu_depth_v2_split]
        #self.nyu_depth_v2_instances = nyu_depth_v2_instances[nyu_depth_v2_split]
        self.nyu_depth_v2_labels = nyu_depth_v2_labels[nyu_depth_v2_split]

        if modalities == "all":
            self.modalities = ALL_MODALITIES
        else:
            self.modalities = modalities

        self.transform = transform

    def __len__(self) -> int:
        return self.nyu_depth_v2_images.shape[0]

    def __getitem__(self, index):

        item = {}

        for m in self.modalities:
            if m == "color":
                item[m] = Image.fromarray(np.int8(np.array(self.nyu_depth_v2_images[index].T)), mode='RGB')
            elif m == "depth":
                item[m] = 0 #Image.fromarray(np.array(self.nyu_depth_v2_depths[index].T))
            elif m == "raw_depth":
                item[m] = 0 #Image.fromarray(np.array(self.nyu_depth_v2_raw_depths[index].T))
            elif m == "semantic":
                item[m] = Image.fromarray(np.uint8(self.palette[np.clip(np.array(self.nyu_depth_v2_labels[index].T), 0, 1999)]), mode='L')
            elif m == "semantic_instance":
                item[m] = 0 #Image.fromarray(np.array(self.nyu_depth_v2_instances[index].T))
            else:
                assert False

        # if self.debug == True:
            
        #     fig = plt.figure(figsize=(7, 7))
        #     rows = 2
        #     columns = 2
        #     fig.add_subplot(rows, columns, 1)
        #     plt.imshow(item["color"])
        #     plt.axis('off')
        #     plt.title("Raw RGB image")
        #     fig.add_subplot(rows, columns, 2)
        #     plt.imshow(item["semantic"])
        #     plt.axis('off')
        #     plt.title("Raw mask")

        #     if self.transforms is not None:
        #         img, target = self.transforms(item["color"], item["semantic"])

        #     fig.add_subplot(rows, columns, 3)
        #     plt.imshow(img.permute(1, 2, 0))
        #     plt.axis('off')
        #     plt.title("Transformed RGB image")
        #     fig.add_subplot(rows, columns, 4)
        #     plt.imshow(target)
        #     plt.axis('off')
        #     plt.title("Transformed mask")
        #     plt.show()
        
        # else:

        if self.transforms is not None:
            img, target = self.transforms(item["color"], item["semantic"])

        return img, target
