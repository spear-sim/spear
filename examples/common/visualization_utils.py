#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np

def get_depth_image_for_visualization(image):

    assert len(image.shape) == 3  # width, height, #channels
    assert image.shape[2] == 4    # RGBA or BGRA

    modified_image = image.copy()[:,:,[0,1,2]] # depth is returned as RGBA

    # discard very large depth values
    max_depth_meters = 20.0
    modified_image = modified_image[:,:,0]
    modified_image = np.clip(modified_image, 0.0, max_depth_meters)

    return modified_image

def get_final_color_image_for_visualization(image):

    assert len(image.shape) == 3  # width, height, #channels
    assert image.shape[2] == 4    # RGBA or BGRA

    return image.copy()[:,:,[2,1,0]] # final_color is returned as BGRA

def get_normal_image_for_visualization(image):

    assert len(image.shape) == 3  # width, height, #channels
    assert image.shape[2] == 4    # RGBA or BGRA

    modified_image = image.copy()[:,:,[0,1,2]] # normal is returned as RGBA

    # discard normals that aren't properly normalized, i.e., length of 1.0
    discard_mask = np.logical_not(np.isclose(np.linalg.norm(modified_image, axis=2), 1.0, rtol=0.001, atol=0.001))
    modified_image = np.clip((modified_image + 1.0) / 2.0, 0.0, 1.0)
    modified_image[discard_mask] = np.nan

    return modified_image

def get_segmentation_image_for_visualization(image):

    assert len(image.shape) == 3  # width, height, #channels
    assert image.shape[2] == 4    # RGBA or BGRA

    return image.copy()[:,:,[2,1,0]] # segmentation is returned as BGRA
