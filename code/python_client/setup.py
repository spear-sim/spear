import setuptools

setuptools.setup(
    name="unrealai",
    version="0.0.1",
    author="rachithp",
    author_email="rachithprakash@gmail.com",
    description="Python client for enabling Artificial Intelligence with Unreal Engine",
    packages=setuptools.find_packages(),
    install_requires=["numpy", "gym", "pyyaml", "opencv-python"],
)
