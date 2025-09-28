#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import shutil
import spear
import trimesh
import wget


# Skip the following meshes from the Stanford 3D Scanning Repository:
#     http://graphics.stanford.edu/pub/3Dscanrep/armadillo/Armadillo.ply.gz          format not recognized by shutil.unpack_archive(...)
#     http://graphics.stanford.edu/data/3Dscanrep/lucy.tar.gz",                      mesh is very large
#     http://graphics.stanford.edu/data/3Dscanrep/xyzrgb/xyzrgb_dragon.ply.gz",      format not recognized by shutil.unpack_archive(...)
#     http://graphics.stanford.edu/data/3Dscanrep/xyzrgb/xyzrgb_statuette.ply.gz"    format not recognized by shutil.unpack_archive(...)

mesh_descs = \
[
    {
        "url": "http://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz",
        "ply_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "ply", "bunny", "reconstruction", "bun_zipper.ply")),
        "gltf_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "gltf", "bunny", "mesh.gltf"))
    },
    {
        "url": "http://graphics.stanford.edu/pub/3Dscanrep/happy/happy_recon.tar.gz",
        "ply_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "ply", "happy_recon", "happy_vrip.ply")),
        "gltf_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "gltf", "happy", "mesh.gltf"))
    },
    {
        "url": "http://graphics.stanford.edu/pub/3Dscanrep/dragon/dragon_recon.tar.gz",
        "ply_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "ply", "dragon_recon", "dragon_vrip.ply")),
        "gltf_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "gltf", "dragon", "mesh.gltf"))
    }
]


if __name__ == '__main__':

    # create output dirs
    download_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "download"))
    ply_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "ply"))

    if os.path.exists(download_dir):
        spear.log("Directory exists, removing: ", download_dir)
        shutil.rmtree(download_dir, ignore_errors=True)

    os.makedirs(download_dir, exist_ok=True)
    os.makedirs(ply_dir, exist_ok=True)

    for mesh_desc in mesh_descs:

        mesh_archive_file = os.path.split(mesh_desc["url"])[1]
        gltf_dir = os.path.split(mesh_desc["gltf_file"])[0]

        download_file = os.path.realpath(os.path.join(download_dir, mesh_archive_file))

        spear.log(f"Downloading {mesh_desc['url']} to {download_file}")
        wget.download(mesh_desc["url"], out=download_file)
        print() # wget doesn't seem to print a newline character when it is finished downloading

        spear.log(f"Extracting {download_file} to {ply_dir}")
        shutil.unpack_archive(download_file, ply_dir)

        spear.log(f"Converting {mesh_desc['ply_file']} to {mesh_desc['gltf_file']}")
        mesh = trimesh.load_mesh(mesh_desc["ply_file"], process=False, validate=False)
        os.makedirs(gltf_dir, exist_ok=True)
        mesh.export(mesh_desc["gltf_file"], "gltf")

    spear.log("Done.")
