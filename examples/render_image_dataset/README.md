# Render a Dataset

In this example application, we demonstrate how to generate random camera poses and render a collection of images. You can run this example as follows.

```console
# generate camera poses
python generate_camera_poses.py

# render images
python run.py

# render images using several optimizations to increase end-to-end throughput (optional)
python run.py --optimized
```

Running `generate_camera_poses.py` will generate a CSV file containing camera poses, and running `run.py` will render an image for each camera pose. Images are saved to `images/non_optimized` or `images/optimized`, so the two modes can be compared, and the average end-to-end frame time (including the time needed to save images to disk) is printed to the terminal.
