import os
from PIL import Image
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import timeit


def resizeImages(size: tuple = (640, 480), path: str = './'):
    """
        Resize the images stored in a directory 
        
        Based on the PIL Image.resize function, this function is especially useful 
        for increasing the size of a collection of images (such as semantic segmentation 
        images). It may however alter the image aspect ratio. The end user is therefore
        responsible for the consistency of the provided size parameter... 
        :param size: The requested size in pixels, as a 2-tuple: (width, height).
        :param path: The path of the folder containing the images.
    """
    for subdir, dirs, files in os.walk(path):
        for file in files:
            image = Image.open(file)
            new_image = image.resize(size)
            new_image.save(file)

def conservativeResizeImages(size: tuple = (640, 480), path: str = './'):
    """
        Resize the images stored in a directory

        Based on the PIL Image.thumbnail function, this function is especially useful 
        for decreasing the size of a collection of images while maintaining their aspect
        ratio. It cannot be used to increase the size of images. 
        :param size: The requested size in pixels, as a 2-tuple: (width, height).
        :param path: The path of the folder containing the images.
    """
    for subdir, dirs, files in os.walk(path):
        for file in files:
            image = Image.open(file)
            new_image = image.thumbnail(size)
            new_image.save(file)

def grayscaleToColorMap(path: str = './'):
    """
        Convert a grayscale image into a colormap image.

        Based on the PIL Image.thumbnail function, this function is especially useful 
        for decreasing the size of a collection of images while maintaining their aspect
        ratio. It cannot be used to increase the size of images. 
        :param size: The requested size in pixels, as a 2-tuple: (width, height).
        :param path: The path of the folder containing the images.
    """
    for subdir, dirs, files in os.walk(path):
        for file in files:
            Image.open(file).convert('P').save(file)
    