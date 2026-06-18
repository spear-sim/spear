#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np


def get_object_ids_bgra_uint8_as_uint32(object_ids_bgra_uint8):
    return object_ids_bgra_uint8.view(np.uint32).reshape(object_ids_bgra_uint8.shape[:2]) & 0x00ffffff

def get_object_ids_rgba_float16_as_uint32(object_ids_rgba_float16):
    object_ids_bgra_float16 = object_ids_rgba_float16[:,:,[2,1,0,3]]
    object_ids_bgra_uint8 = np.round(object_ids_bgra_float16*255.0).astype(np.uint8)
    return get_object_ids_bgra_uint8_as_uint32(object_ids_bgra_uint8=object_ids_bgra_uint8)

def tone_map_hypersim(image, mask, as_dict=None):

    image = image.astype(np.float64)

    gamma                             = 1.0/2.2   # standard gamma correction exponent
    inv_gamma                         = 1.0/gamma
    percentile                        = 90        # we want this percentile brightness value in the unmodified image...
    brightness_nth_percentile_desired = 0.8       # ...to be this bright after scaling

    if np.count_nonzero(mask) == 0:
        scale = 1.0 # if there are no valid pixels, then set scale to 1.0
    else:
        brightness      = 0.3*image[:,:,0] + 0.59*image[:,:,1] + 0.11*image[:,:,2] # "CCIR601 YIQ" method for computing brightness
        brightness_mask = brightness[mask]

        eps                               = 0.0001 # if the nth percentile brightness value in the unmodified image is less than this, set the scale to 0.0 to avoid divide-by-zero
        brightness_nth_percentile_current = np.percentile(brightness_mask, percentile)

        if brightness_nth_percentile_current < eps:
            scale = 0.0
        else:

            # Snavely uses the following expression in the code at https://github.com/snavely/pbrs_tonemapper/blob/master/tonemap_rgbe.py:
            # scale = np.exp(np.log(brightness_nth_percentile_desired)*inv_gamma - np.log(brightness_nth_percentile_current))
            #
            # Our expression below is equivalent, but is more intuitive, because it follows more directly from the expression:
            # (scale*brightness_nth_percentile_current)^gamma = brightness_nth_percentile_desired

            scale = np.power(brightness_nth_percentile_desired, inv_gamma) / brightness_nth_percentile_current

    image_tone_map = np.power(np.maximum(scale*image, 0.0), gamma)

    if as_dict is None:
        return image_tone_map
    else:
        assert as_dict
        return {"image_tone_map": image_tone_map, "scale": scale, "gamma": gamma}
