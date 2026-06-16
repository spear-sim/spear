#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import imageio_ffmpeg
import os
import subprocess
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
parser.add_argument("--preview", action="store_true")
args = parser.parse_args()

# Main quality-vs-size setting: the H.264 constant rate factor. Lower means higher quality and larger files, higher
# means lower quality and smaller files; 23 is libx264's default and ~18 is visually near-lossless.
crf = 23

# In preview mode (--preview) we use a more aggressive (higher) constant rate factor, for smaller files at lower
# quality.
preview_crf = 35

# Frames per second of the generated videos.
frame_rate = 30


def process_scene():

    # Encode each camera path's rendered images (one subdirectory per path, written by
    # generate_free_space_camera_path_images.py) into an H.264 video. We call the ffmpeg binary bundled with the
    # imageio-ffmpeg package, so no system ffmpeg installation is required. Preview mode reads the preview images and
    # uses a more aggressive constant rate factor; final mode reads the full-resolution images.
    if args.preview:
        images_dir_prefix = "preview"
        video_crf = preview_crf
    else:
        images_dir_prefix = "final"
        video_crf = crf

    images_dir = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_camera_path_images", images_dir_prefix))
    assert os.path.exists(images_dir)
    videos_dir = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_camera_path_videos", images_dir_prefix))
    os.makedirs(videos_dir, exist_ok=True)

    ffmpeg_executable = imageio_ffmpeg.get_ffmpeg_exe()

    path_images_dirs = sorted([ path_images_dir for path_images_dir in glob.glob(os.path.join(images_dir, "*")) if os.path.isdir(path_images_dir) ])
    for path_images_dir in path_images_dirs:

        # The images in each path's subdirectory are named by their zero-padded frame index (see
        # generate_free_space_camera_path_images.py), which ffmpeg reads as a numbered sequence.
        video_file = os.path.realpath(os.path.join(videos_dir, f"{os.path.basename(path_images_dir)}.mp4"))
        spear.log("Writing video file: ", video_file)
        subprocess.run([
            ffmpeg_executable, "-y", "-loglevel", "warning",
            "-framerate", str(frame_rate),
            "-start_number", "0",
            "-i", os.path.join(path_images_dir, "%04d.png"),
            "-c:v", "libx264",
            "-crf", str(video_crf),
            "-pix_fmt", "yuv420p",
            video_file], check=True)

    spear.log("Done.")


if __name__ == "__main__":
    process_scene()
