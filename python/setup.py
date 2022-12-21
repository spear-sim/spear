#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import setuptools

setuptools.setup(
    name="spear-sim",
    version="0.1.0",
    author="",
    author_email="",
    description="",
    packages=["spear"],
    install_requires=["ffmpeg-python==0.2.0", "gym==0.21.0", "matplotlib==3.6.2", "numpy==1.22.3", "opencv-python==4.5.5.64", "pandas==1.5.1", "psutil==5.9.0", "tflite-runtime>=2.5.0,<=2.7.0", "yacs==0.1.8"]
)

