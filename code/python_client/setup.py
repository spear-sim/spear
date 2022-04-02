import setuptools

setuptools.setup(
    name="interiorsim",
    version="0.0.1",
    author="",
    author_email="",
    description="Python client for enabling Artificial Intelligence with Unreal Engine",
    packages=setuptools.find_packages(),
    install_requires=["numpy", "gym", "psutil", "yacs==0.1.8"],
)
