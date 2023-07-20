#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import setuptools
import sys

install_requires_common = ["ffmpeg-python==0.2.0", "gym==0.22.0", "matplotlib==3.6.2", "numpy==1.22.3", "opencv-python==4.5.5.64", "pandas==1.5.1", "psutil==5.9.0", "yacs==0.1.8"]

if sys.platform == "win32":
    install_requires_platform = ["tensorflow==2.9.3"]
elif sys.platform == "darwin":
    install_requires_platform = ["tensorflow-macos==2.9.2"]
elif sys.platform == "linux":
    install_requires_platform = ["tensorflow==2.9.3"]
else:
    assert False

install_requires = install_requires_common + install_requires_platform

setuptools.setup(
    name="spear-sim",
    version="0.3.0",
    author="",
    author_email="",
    description="",
    packages=["spear"],
    install_requires=install_requires
)
