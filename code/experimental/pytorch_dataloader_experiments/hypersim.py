import h5py
import numpy as np
import os
import pandas as pd
import torch

ALL_MODALITIES = [
    { "name" : "depth_meters",         "render_pass" : "geometry" },
    { "name" : "render_entity_id",     "render_pass" : "geometry" },
    { "name" : "semantic",             "render_pass" : "geometry" },
    { "name" : "semantic_instance",    "render_pass" : "geometry" },
    { "name" : "normal_bump_cam",      "render_pass" : "geometry" },
    { "name" : "normal_bump_world",    "render_pass" : "geometry" },
    { "name" : "normal_cam",           "render_pass" : "geometry" },
    { "name" : "normal_world",         "render_pass" : "geometry" },
    { "name" : "position",             "render_pass" : "geometry" },
    { "name" : "tex_coord",            "render_pass" : "geometry" },
    { "name" : "color",                "render_pass" : "final" },
    { "name" : "diffuse_illumination", "render_pass" : "final" },
    { "name" : "diffuse_reflectance",  "render_pass" : "final" },
    { "name" : "residual",             "render_pass" : "final" } ]

class ToneMap():

    def __init__(self, modality, gamma=1.0/2.2, percentile=90, brightness_nth_percentile_desired=0.8):
        
        self.modality                          = modality
        self.gamma                             = gamma                             # standard gamma correction exponent
        self.inv_gamma                         = 1.0/gamma
        self.percentile                        = percentile                        # we want this percentile brightness value in the unmodified image...
        self.brightness_nth_percentile_desired = brightness_nth_percentile_desired # ...to be this bright after scaling
    
    def __call__(self, item):
 
        rgb_color        = item[self.modality].astype(np.float32)
        render_entity_id = item["render_entity_id"].astype(np.int32)
        
        # compute brightness according to "CCIR601 YIQ" method, use CGIntrinsics strategy for tonemapping, see [1,2,3]
        # [1] https://github.com/snavely/pbrs_tonemapper/blob/master/tonemap_rgbe.py
        # [2] https://github.com/apple/ml-hypersim/blob/master/code/python/tools/scene_generate_images_tonemap.py
        # [3] https://landofinterruptions.co.uk/manyshades

        assert np.all(render_entity_id != 0)

        gamma                             = self.gamma
        inv_gamma                         = self.inv_gamma
        percentile                        = self.percentile
        brightness_nth_percentile_desired = self.brightness_nth_percentile_desired

        valid_mask = render_entity_id != -1

        if np.count_nonzero(valid_mask) == 0:
            scale = 1.0 # if there are no valid pixels, then set scale to 1.0
        else:
            brightness       = 0.3*rgb_color[:,:,0] + 0.59*rgb_color[:,:,1] + 0.11*rgb_color[:,:,2] # "CCIR601 YIQ" method for computing brightness
            brightness_valid = brightness[valid_mask]

            eps                               = 0.0001 # if the kth percentile brightness value in the unmodified image is less than this, set the scale to 0.0 to avoid divide-by-zero
            brightness_nth_percentile_current = np.percentile(brightness_valid, percentile)

            if brightness_nth_percentile_current < eps:
                scale = 0.0
            else:

                # Snavely uses the following expression in the code at https://github.com/snavely/pbrs_tonemapper/blob/master/tonemap_rgbe.py:
                # scale = np.exp(np.log(brightness_nth_percentile_desired)*inv_gamma - np.log(brightness_nth_percentile_current))
                #
                # Our expression below is equivalent, but is more intuitive, because it follows more directly from the expression:
                # (scale*brightness_nth_percentile_current)^gamma = brightness_nth_percentile_desired

                scale = np.power(brightness_nth_percentile_desired, inv_gamma) / brightness_nth_percentile_current

        rgb_color_tm = np.power(np.maximum(scale*rgb_color,0), gamma)

        item[self.modality] = np.clip(rgb_color_tm, 0, 1)
        
        return item

class HypersimDataset(torch.utils.data.Dataset):

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
        
        split_file = os.path.join(data_dir, "metadata_images_split_scene_v1.csv")
        df = pd.read_csv(split_file).rename_axis("frame_guid")
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
            hdf5_file = os.path.join(self.data_dir, i["scene_name"], "images", "scene_"+i["camera_name"]+"_"+m["render_pass"]+"_hdf5", "frame.%04d.%s.hdf5"%(i["frame_id"],m["name"]))
            with h5py.File(hdf5_file, "r") as f: img = f["dataset"][:]
            item[m["name"]] = img

        if self.transform is not None:
            item = self.transform(item)

        return item
    
    def __len__(self):
        if self.num_items is not None:
            return self.num_items
        else:
            return len(self.df_split)
