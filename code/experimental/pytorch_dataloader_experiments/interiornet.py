# import h5py
import numpy as np
import os
import pandas as pd
from PIL import Image
import torch
from tqdm import tqdm
# from multiprocessing import Process, Manager


ALL_MODALITIES = [
    { "name" : "albedo",                       "dir" : "albedo0",                       "ext" : ".png" },
    { "name" : "color",                        "dir" : "cam0",                          "ext" : ".png" },
    { "name" : "depth",                        "dir" : "depth0",                        "ext" : ".png" },
    { "name" : "illumination",                 "dir" : "illumination0",                 "ext" : ".png" },
    { "name" : "semantic",                     "dir" : "label0",                        "ext" : "_nyu.png" },
    { "name" : "semantic_instance",            "dir" : "label0",                        "ext" : "_instance.png" },
    { "name" : "normal",                       "dir" : "normal0",                       "ext" : ".png" },
    { "name" : "random_lighting",              "dir" : "random_lighting_cam0",          "ext" : ".png" },
    { "name" : "random_lighting_illumination", "dir" : "random_lighting_illumination0", "ext" : ".png" } ]

class InteriorNetDataset(torch.utils.data.Dataset):

    def __init__(
        self,
        data_dir,
        split,
        modalities="all",
        transform=None,
        num_items=None):

        assert split in ["train", "val", "test"]
        assert modalities == "all" or set(modalities) <= set([m["name"] for m in ALL_MODALITIES])
        
        self.data_dir = data_dir
        
        split_file = os.path.join(data_dir, "interiornet_50k_split_v2.csv")
        df = pd.read_csv(split_file).rename_axis("image_guid")
        self.df_split = df[ df["split_partition_name"] == split ].reset_index()

        if modalities == "all":
            self.modalities = ALL_MODALITIES
        else:
            self.modalities = [ m for m in ALL_MODALITIES if m["name"] in modalities ]

        self.transform = transform
        self.num_items = num_items
        
    def __getitem__(self, index):
        
        item = {}
        i = self.df_split.iloc[index]
        
        for m in self.modalities:
            img_file = os.path.join(self.data_dir, i["scene_name"] + "_" + i["scene_type"], m["dir"], "data", "%d"%i["image_id"] + m["ext"])
            img = np.asarray(Image.open(img_file)).copy()
            item[m["name"]] = img

        if self.transform is not None:
            item = self.transform(item)

        return item
    
    def __len__(self):
        if self.num_items is not None:
            return self.num_items
        else:
            return len(self.df_split)

class InteriorNetMultiLabelDataset(torch.utils.data.Dataset):
    
    def __init__(
        self,
        data_dir,
        split,
        modalities="all",
        transform=None,
        num_items=None,
        ffcv_generation=False):
        self.ffcv_generation = ffcv_generation

        assert split in ["train", "test"]
        modalities = ['color', 'semantic']
        assert modalities == "all" or set(modalities) <= set([m["name"] for m in ALL_MODALITIES])
        
        self.data_dir = data_dir
        
        split_file = os.path.join(data_dir, "interiornet_50k_split_v2.csv")
        # split_file = os.path.join(data_dir, "interiornet_50k_split_temp.csv")
        df = pd.read_csv(split_file).rename_axis("image_guid")
        if split == "train":
            self.df_split = df[ df["split_partition_name"] == split ].reset_index()
        else:
            self.df_split = pd.concat([df[ df["split_partition_name"] == 'val' ], df[ df["split_partition_name"] == 'test' ]]).reset_index()

        self.modalities = {m["name"]: m for m in ALL_MODALITIES if m["name"] in modalities }

        self.transform = transform
        self.num_items = num_items

        self.mapping =  [0, 12, 5, 6, 1, 4, 9, 10, 12, 13, 6, 8, 6, 13, 10, 6, 13, 6, 7, 7, 5, 7, 3, 2, 6, 11, 7, 7, 7, 7, 7, 7, 6, 7, 7, 7, 7, 7, 7, 6, 7]
        # nyu 40 to 13, but we don't want to use objects (too broad), and ceiling/floor/wall (shows up almost every image)
        # classes = ['bed', 'books', 'ceiling', 'chair', 'floor', 'furniture', 'objects', 'picture', 'sofa', 'table', 'tv', 'wall', 'window']
        # classes = ['bed', 'chair/sofa', 'picture', 'table', 'tv', 'window']

    def __getitem__(self, index):
        i = self.df_split.iloc[index]
        
        m = self.modalities['color']
        img_file = os.path.join(self.data_dir, i["scene_name"] + "_" + i["scene_type"], m["dir"], "data", "%d"%i["image_id"] + m["ext"])
        # image = np.asarray(Image.open(img_file)).copy()
        image = Image.open(img_file).convert('RGB')
        if self.transform is not None:
            image = self.transform(image)
        
        m = self.modalities['semantic']
        label_file = os.path.join(self.data_dir, i["scene_name"] + "_" + i["scene_type"], m["dir"], "data", "%d"%i["image_id"] + m["ext"])
        label_image = np.asarray(Image.open(label_file)).copy()

        classes = list(set(label_image.flatten().tolist()))
        classes = list(set([self.mapping[c] for c in classes if c >= 0]))
        classes = [c-1 for c in classes if c > 0]
        classes = [c for c in classes if c != 11] # remove wall
        classes = [c-1 if c > 11 else c for c in classes] # remove wall
        # combine sofa with chair
        if 8 in classes and 3 not in classes:
            classes.append(3)
        classes = [c for c in classes if c != 8] # combine sofa with chair
        classes = [c-1 if c > 8 else c for c in classes] # combine sofa with chair
        classes = [c for c in classes if c != 6] # remove objects
        classes = [c-1 if c > 6 else c for c in classes] # remove objects
        classes = [c for c in classes if c != 5] # remove furniture
        classes = [c-1 if c > 5 else c for c in classes] # remove furniture
        classes = [c for c in classes if c != 4] # remove floor
        classes = [c-1 if c > 4 else c for c in classes] # remove floor
        classes = [c for c in classes if c != 2] # remove ceiling
        classes = [c-1 if c > 2 else c for c in classes] # remove ceiling
        classes = [c for c in classes if c != 1] # remove books
        classes = [c-1 if c > 1 else c for c in classes] # remove books
        # label = np.zeros(8)
        label = np.zeros(6)
        for c in classes:
            label[c] = 1
        label = torch.tensor(label).float()
        if self.ffcv_generation:
            label = label.numpy().astype('float32')
        return image, label
        

    def __len__(self):
        if self.num_items is not None:
            return self.num_items
        else:
            return len(self.df_split)


def checker(broken, index=None):
        
    data_dir = '/gscratch/efml/apf1/active_segmentation/dataset_util/interiornet/interiornet_data'
    split_file = os.path.join(data_dir, "interiornet_50k_split_v2.csv")
    df = pd.read_csv(split_file).rename_axis("image_guid")
    df_split = df
    
    if index is None:
        indices = np.arange(len(df))
    else:
        indices = [index]
    for index in tqdm(indices):
        i = df_split.iloc[index]
        
        modalities = ['color', 'semantic']
        modalities = {m["name"]: m for m in ALL_MODALITIES if m["name"] in modalities }
        m = modalities['color']
        img_file = os.path.join(data_dir, i["scene_name"] + "_" + i["scene_type"], m["dir"], "data", "%d"%i["image_id"] + m["ext"])
        try:
            image = Image.open(img_file).convert('RGB')
            image.close()
        except:
            print(img_file)
            broken.append(img_file)
        
        m = modalities['semantic']
        label_file = os.path.join(data_dir, i["scene_name"] + "_" + i["scene_type"], m["dir"], "data", "%d"%i["image_id"] + m["ext"])
        try:
            label_image = Image.open(label_file)
            label_image.close()
        except:
            print(label_file)
            broken.append(label_file)
    if index is None:
        with open('broken.txt', mode='wt', encoding='utf-8') as myfile:
            myfile.write('\n'.join(L))
            myfile.write('\n')


if __name__ =="__main__":
    '''
    with Manager() as manager:
        L = manager.list()  # <-- can be shared between processes.
        processes = []
        for i in tqdm(range(50000)):
            p = Process(target=checker, args=(L,i))  # Passing the list
            p.start()
            processes.append(p)
        for p in processes:
            p.join()
        # print L
    with open('broken.txt', mode='wt', encoding='utf-8') as myfile:
        myfile.write('\n'.join(L))
        myfile.write('\n')
    '''
    checker([])
