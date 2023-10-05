import multiprocessing as mp
if mp.current_process().name == 'MainProcess':
    import bpy
import argparse
from dataclasses import dataclass
import enum
import os
import shutil
import subprocess
import copy
from collections import deque
import json
import xml.etree.ElementTree as ET
import numpy as np
import transforms3d.quaternions as txq
import trimesh
from itertools import combinations


osp = os.path

use_CoACD = True
if use_CoACD:
    import coacd

# Find the v4.0 executable in the system path.
# Note trimesh has not updated their code to work with v4.0 which is why we do not use
# their `convex_decomposition` function.
_VHACD_EXECUTABLE = shutil.which("TestVHACD")

# Names of the V-HACD output files.
_VHACD_OUTPUTS = ["decomp.obj", "decomp.stl", "decomp.mtl"]

class FillMode(enum.Enum):
    FLOOD = enum.auto()
    SURFACE = enum.auto()
    RAYCAST = enum.auto()

@dataclass
class AssetProperties:
    name: str = ""
    id: int = 0
    moving: bool = False
    max_hulls: int = 0
    voxel_res: int = 0
    hull_vert_count: int = 0

@dataclass
class VhacdArgs:
    enable: bool = True
    """enable convex decomposition using V-HACD"""
    max_output_convex_hulls: int = 8
    """maximum number of output convex hulls for a single mesh"""
    voxel_resolution: int = 400000
    """total number of voxels to use (higher voxel resolutions will capture finer details but be more computationally intensive)"""
    volume_error_percent: float = 0.001
    """volume error allowed as a percentage (lower values will result in higher number of convex hulls. Allow detecting regions that are already convex and stop spend time on them)"""
    max_recursion_depth: int = 64
    """maximum recursion depth"""
    disable_shrink_wrap: bool = False
    """do not shrink wrap output to source mesh to better match the original mesh"""
    fill_mode: FillMode = FillMode.FLOOD
    """fill mode"""
    max_hull_vert_count: int = 512
    """maximum number of vertices in the output convex hull (the higher the greater the computation effort on the physics engine side)"""
    disable_async: bool = False
    """do not run asynchronously"""
    min_edge_length: int = 2
    """minimum size of a voxel edge (default is usually fine)"""
    split_hull: bool = False
    """try to find optimal split plane location (experimental)"""

@dataclass
class CoacdArgs:
    threshold: float = 0.02
    """Termination criteria in [0.01, 1] (0.01: most fine-grained; 1: most coarse)"""
    max_convex_hull: int = -1
    """Maximum number of convex hulls in the result, -1 for no limit, works only when merge is enabled."""
    preprocess_mode: str = "auto"
    """No remeshing before running CoACD. Only suitable for manifold input."""
    preprocess_resolution: int = 100
    """Preprocessing resolution."""
    resolution: int = 2000
    """Surface samping resolution for Hausdorff distance computation."""
    mcts_nodes: int = 40
    """Number of cut candidates for MCTS."""
    mcts_iterations: int = 500
    """Number of MCTS iterations."""
    mcts_max_depth: int = 7
    """Maximum depth for MCTS search."""
    pca: bool = False
    """Use PCA to align input mesh. Suitable for non-axis-aligned mesh."""
    merge: bool = True
    """If merge is enabled, try to reduce total number of parts by merging."""
    seed: int = 0
    """Random seed."""

@dataclass
class ACD_Args:
    use_coacd : bool
    coacd_args: CoacdArgs
    vhacd_args: VhacdArgs

def name_in_names(name, names):
  for n in names:
      if name in n:
          return n
  return None


def mujoco_xml_populate_objects(object_folder, output_folder, scene_path, cvx_folder, xyz, quat, decompose_in_bodies):
    print("populating xml file")
    _, body_name = os.path.split(object_folder)
    
    prefix = object_folder.replace(output_folder, '')
    if prefix[0] == osp.sep:
        prefix = prefix[1:]
    prefix = prefix.split(osp.sep)[1:]
    prefix = '.'.join(prefix)

    # write assets file
    root = ET.Element('mujoco', {'model': body_name})
    # asset = ET.SubElement(root, 'asset')
    # object_path = osp.join(*(object_folder.split(osp.sep)[5:] or ['']))
    object_path = object_folder.replace(scene_path, '')
    if object_path[0] == osp.sep:
        object_path = object_path[1:]
    object_name = osp.split(object_folder)[-1]
    
    # ET.SubElement(root, 'mesh', {'file': osp.join(object_path, f'{object_name}_cvx.obj')})
    # ET.SubElement(root, 'mesh', {'file': f'output/{object_cat[0]}/{obj_name[0]}/{obj_name[0]+"_cvx"}.obj'})
    _, _, mesh_objects = next(os.walk(osp.join(object_folder, cvx_folder)))
    for mesh_object in sorted(mesh_objects):
        split_file_name = osp.basename(mesh_object)
        split_name, ext = osp.splitext(split_file_name)
        if ext != '.obj':
            continue
        ET.SubElement(root, 'mesh',
                      {'file': osp.join(object_path, cvx_folder, split_file_name), 'name': f'{prefix}.{split_name}'}
        )
        # ET.SubElement(root, 'mesh', {'file': f'output/{object_cat[0]}/{obj_name[0]}/cvx/{file_name[0]}.obj'})
    
    filename = osp.join(object_folder, cvx_folder, f'{body_name}_assets.xml')
    tree = ET.ElementTree(root)
    ET.indent(tree, space="\t", level=0)
    tree.write(filename)
    print(f'{filename} written')

    # write body file
    root = ET.Element('mujoco', {'model': body_name})
    file_name = osp.splitext(object_folder)
    # TODO(samarth) check if both cases are same
    if decompose_in_bodies:
        _, _, mesh_objects = next(os.walk(osp.join(object_folder, cvx_folder)))
        for mesh_object in sorted(mesh_objects):
            file_name = os.path.splitext(osp.basename(mesh_object))
            if file_name[1] != '.obj':
                continue
            if not file_name[0].startswith("VolumeError"):
                body_attributes = {
                    'name': file_name[0],
                    'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                    'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
                }
                # body = ET.SubElement(root, 'body', body_attributes)
                geom_attributes = {
                    'name': f'{prefix}.{file_name[0]}_vis',
                    'mesh': f'{prefix}.{file_name[0]}',
                    'class': 'visual',
                    'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                    'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}', 
                }
                ET.SubElement(root, 'geom', geom_attributes)
                geom_attributes = {
                    'name': f'{prefix}.{file_name[0]}_col',
                    'mesh': f'{prefix}.{file_name[0]}',
                    'class': 'collision',
                    'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                    'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}', 
                }
                ET.SubElement(root, 'geom', geom_attributes)
    else:
        body_attributes = {
            'name': body_name,
            'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
            'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
        }
        # body = ET.SubElement(root, 'body', body_attributes)
        # ET.SubElement(body, 'geom', {'name': f'{body_name}_vis', 'mesh': f'{body_name}_cvx', 'class': 'visual'})
        _, _, mesh_objects = next(os.walk(osp.join(object_folder, cvx_folder)))
        for mesh_object in sorted(mesh_objects):
            file_name = os.path.splitext(osp.basename(mesh_object))
            if 'obj' not in file_name[1]:
                continue
            if not file_name[0].startswith("VolumeError"):
                geom_attributes = {
                    'name': f'{prefix}.{file_name[0]}_vis',
                    'mesh': f'{prefix}.{file_name[0]}',
                    'class': 'visual',
                    'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                    'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
                }
                ET.SubElement(root, 'geom', geom_attributes)
                geom_attributes = {
                    'name': f'{prefix}.{file_name[0]}_col',
                    'mesh': f'{prefix}.{file_name[0]}',
                    'class': 'collision',
                    'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                    'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
                }
                ET.SubElement(root, 'geom', geom_attributes)
    filename = osp.join(object_folder, cvx_folder, f'{body_name}_body.xml')
    tree = ET.ElementTree(root)
    ET.indent(tree, space="\t", level=0)
    tree.write(filename)
    print(f'{filename} written')
    print(f"Done !")
    print(f"----------------------------------------------------------------------------")


def decompose_convex(args):
    fullpath, output_folder, scene_path, cvx_dir, acd_args, xyz, quat, decompose_in_bodies, rerun = args
    obj_dir, obj_file = os.path.split(fullpath)

    # Copy the obj file to the temporary directory.
    decomposed = False
    cvx_path = os.path.join(obj_dir, cvx_dir)
    if rerun and osp.isdir(cvx_path):
        decomposed = True
    else:
        os.makedirs(cvx_path, exist_ok=True)
        shutil.copy(fullpath, cvx_path)

        obj_filename = osp.join(cvx_path, obj_file)
        obj_name = os.path.splitext(obj_file)[0]
        
        if acd_args.use_coacd:    
            # Call CoACD.
            mesh = trimesh.load(obj_filename, force="mesh")
            mesh = coacd.Mesh(mesh.vertices, mesh.faces)
            result = coacd.run_coacd(
                mesh,
                threshold=acd_args.coacd_args.threshold,
                max_convex_hull=acd_args.coacd_args.max_convex_hull,
                preprocess_mode=acd_args.coacd_args.preprocess_mode,
                preprocess_resolution=acd_args.coacd_args.preprocess_resolution,
                resolution=acd_args.coacd_args.resolution,
                mcts_nodes=acd_args.coacd_args.mcts_nodes,
                mcts_iterations=acd_args.coacd_args.mcts_iterations,
                mcts_max_depth=acd_args.coacd_args.mcts_max_depth,
                pca=acd_args.coacd_args.pca,
                merge=acd_args.coacd_args.merge,
                seed=acd_args.coacd_args.seed,
                )

            mesh_parts = []
            for vs, fs in result:
                # remove faces with duplicate vertices
                fs = [f for f in fs if len(set(f)) == 3]
                # remove duplicate faces
                fs_set = set()
                final_fs = []
                for f in fs:
                    ff = frozenset(f)  # frozenset() also sorts
                    if ff in fs_set:
                        fs_set.remove(ff)
                    else:
                        fs_set.add(ff)
                        final_fs.append(f)
                mesh = trimesh.Trimesh(vs, final_fs)
                if mesh.is_watertight:
                  mesh_parts.append(mesh)

            object_assembled = trimesh.Scene()
            np.random.seed(0)
            i = 0
            for p in mesh_parts:
                p.visual.vertex_colors[:, :3] = (np.random.rand(3) * 255).astype(np.uint8)
                object_assembled.add_geometry(p)
                object_part = trimesh.Scene()
                object_part.add_geometry(p)
                object_part.export(osp.join(cvx_path, f'{obj_name}_{str(i)}.obj'))
                i+=1
            if not object_assembled.is_empty:
                object_assembled.export(osp.join(obj_dir, f'{obj_name}_cvx.obj'))
                decomposed = True
        
        else:
            
            if not acd_args.vhacd_args.enable:
                return False

            if _VHACD_EXECUTABLE is None:
                print(
                    "V-HACD was enabled but not found in the system path. Either install it "
                    "manually or run `bash install_vhacd.sh`. Skipping decomposition"
                )
                return False

            # Call V-HACD, suppressing output.
            ret = subprocess.run(
                [
                    f"{_VHACD_EXECUTABLE}",
                    osp.join(cvx_dir, obj_file),
                    "-i",
                    "stl",
                    "-o",
                    "obj",
                    "-h",
                    f"{acd_args.vhacd_args.max_output_convex_hulls}",
                    "-r",
                    f"{acd_args.vhacd_args.voxel_resolution}",
                    "-e",
                    f"{acd_args.vhacd_args.volume_error_percent}",
                    "-d",
                    f"{acd_args.vhacd_args.max_recursion_depth}",
                    "-s",
                    f"{int(not acd_args.vhacd_args.disable_shrink_wrap)}",
                    "-f",
                    f"{acd_args.vhacd_args.fill_mode.name.lower()}",
                    "-v",
                    f"{acd_args.vhacd_args.max_hull_vert_count}",
                    "-a",
                    f"{int(not acd_args.vhacd_args.disable_async)}",
                    "-l",
                    f"{acd_args.vhacd_args.min_edge_length}",
                    "-p",
                    f"{int(acd_args.vhacd_args.split_hull)}",
                ],
                check=True,
                cwd=obj_dir,
            )
                    
            if ret.returncode != 0:
                print(f"V-HACD failed on {fullpath}")
                return False
            
            decomposed = True
            # VHACD not being able to generate a convex hull decoposition (while not throwing any error) is a
            # good indicator of the level of fucked-upness of an asset. For now, we simply discard the asset.
            os.rename(osp.join(obj_dir, _VHACD_OUTPUTS[0]), osp.join(obj_dir, f'{obj_name}_{cvx_dir}.obj'))
            os.remove(osp.join(obj_dir, _VHACD_OUTPUTS[1]))
        
        print(f"Removing {obj_filename}")
        os.remove(obj_filename)
        
    if decomposed:
        mujoco_xml_populate_objects(obj_dir, output_folder, scene_path, cvx_dir, xyz, quat, decompose_in_bodies)
    else:  # delete whole directory
        shutil.rmtree(obj_dir)
    return True


def assemble_articulated_object_files(args):
    obj_name, obj_dir, scene_path, nodes_info, joints_info, moving, parent_name_suffix = args
    full_obj_name = f'{obj_name}{parent_name_suffix}'
    root = ET.Element('mujoco')
    asset = ET.SubElement(root, 'asset')
    worldbody = ET.SubElement(root, 'worldbody')
    contact = ET.SubElement(root, 'contact')

    q = deque([(obj_dir, worldbody, ''), ])
    sibling_dir_groups = []
    dir2bodyname = {}
    leaf_dirs = set()
    while len(q):
        dir, parent, name_prefix = q.popleft()
        parent_name = parent.get('name', None)
        # remove prefixes to enable search in joints_info dict
        if parent_name is not None:
            parent_name = parent_name.split('.')[-1] if parent_name.count('.') > 1 else parent_name.split('.')[0]
        name = osp.split(dir)[-1]
        geom_parent = parent
        
        # check if body has a joint with parent
        joint = list(filter(lambda j: (j['parent'] == parent_name) and (j['child'] == name), joints_info.values()))
        
        if (len(joint) == 1) or (parent_name is None):
            body_name = f'{name_prefix}{name}{parent_name_suffix}'
            body_attributes = {
                'name': body_name,
                'pos': '{:f} {:f} {:f}'.format(*nodes_info[name]['pos']),
                'quat': '{:f} {:f} {:f} {:f}'.format(*nodes_info[name]['quat'])
            }
            body = ET.SubElement(parent, 'body', body_attributes)
            if len(joint) == 1:
                joint_name = f'{name_prefix}{joint[0]["name"]}{parent_name_suffix}'
                joint_attributes = {
                    'name': joint_name,
                    'type': joint[0]['type'],
                    'pos': '{:.5f} {:.5f} {:.5f}'.format(*joint[0]['pos']),
                    'axis': '{:.5f} {:.5f} {:.5f}'.format(*joint[0]['axis']),
                    'range': '{:.5f} {:.5f}'.format(*joint[0]['range'])
                }
                ET.SubElement(body, 'joint', joint_attributes)
            elif moving:
                ET.SubElement(body, 'freejoint', {'name': f'{obj_name}.freejoint'})
            js = list(filter(lambda j: j['parent'] == name, joints_info.values()))
            if len(js) == 0:
                leaf_dirs.add(dir)
            dir2bodyname[dir] = body_name
            geom_parent = body
            name_prefix = f'{name_prefix}{name}{parent_name_suffix}.'
            parent_name_suffix = ''
        elif len(joint) >= 2:
            raise AssertionError(f'More than one joint found with parent {parent_name} and child {name}')
        
        # check if body has its geom(s)
        cvx_dir = osp.join(dir, 'cvx')
        if os.path.isdir(cvx_dir):
            include_dir = cvx_dir.replace(scene_path, '')
            if include_dir[0] == '/':
                include_dir = include_dir[1:]
            ET.SubElement(geom_parent, 'include', {'file': osp.join(include_dir, f'{name}_body.xml')})
            ET.SubElement(asset, 'include', {'file': osp.join(include_dir, f'{name}_assets.xml')})
        
        # explore children
        _, child_dirs, _ = next(os.walk(dir))
        child_dirs = [osp.join(dir, child_dir) for child_dir in sorted(child_dirs) if child_dir != 'cvx']
        q.extendleft([(child_dir, geom_parent, name_prefix) for child_dir in child_dirs])
        if len(child_dirs):
            sibling_dir_groups.append(child_dirs)

    # contact exclude tags for leaf sibling bodies
    for sibling_dir_group in sibling_dir_groups:
        sdg = [s for s in sibling_dir_group if s in leaf_dirs]
        for body_pair in combinations(sdg, 2):
            ET.SubElement(contact, 'exclude', {'body1': dir2bodyname[body_pair[0]], 'body2': dir2bodyname[body_pair[1]]})
        for dir in sdg:
            ET.SubElement(contact, 'exclude', {'body1': dir2bodyname[dir], 'body2': full_obj_name})
    
    # write MJCF
    tree = ET.ElementTree(root)
    ET.indent(tree, space="\t", level=0)
    filename = os.path.join(obj_dir, f'{obj_name}.xml')
    tree.write(filename)
    print(f'{filename} written')


class AssetProcessor:
    def __init__(self, scene_path: str, n_workers: int, rerun: bool, include_objects: str) -> None:
        self.n_workers = min(max(1, n_workers), mp.cpu_count()-1)
        self.rerun = rerun
        ue_export_path = osp.join(scene_path, 'ue_export')
        self.input_gltf_filenames = {}
        for filename in next(os.walk(ue_export_path))[2]:
            if not filename.endswith('.gltf'):
                continue
            if include_objects and not any([i in filename for i in include_objects.split(',')]):
                continue
            joint_filename = filename.replace('.gltf', '_joints.json')
            self.input_gltf_filenames[os.path.join(ue_export_path, filename)] = os.path.isfile(os.path.join(ue_export_path, joint_filename))
        self.scene_path = osp.normpath(scene_path)
        self.scene_xml_file = osp.join(self.scene_path, "scene.xml") 
        self.output_folder = osp.join(self.scene_path, "output") 
        
        self.mobility_exception_refs = [""]
        
        if not self.rerun:
            shutil.rmtree(self.output_folder, ignore_errors=True)
            try:
                os.remove(self.scene_xml_file)
            except:
                pass

        # Clear scene
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()
        
        # Import the .gltf scene files
        self.object_has_joint = {}
        for gltf_filename, has_joint in self.input_gltf_filenames.items():
          try:
              print(f'importing {gltf_filename}')
              bpy.ops.import_scene.gltf(filepath=gltf_filename)
              object_name = bpy.context.object.name.split('.')[0]
              self.object_has_joint[object_name] = has_joint
          except RuntimeError:
              print('Failed')

        # whether each object is moving or not
        with open(osp.join(ue_export_path, 'actors_information.json'), 'r') as f:
            self.actors_information = json.load(f)
    
    def get_object_type(self, name):
        return "Unknown"
    
    def generate_mujoco_scene(self):
        template_root = ET.parse(osp.join('Python_Export_Pipeline', 'scene_include.xml')).getroot()
        root = ET.Element('mujoco', {'model': osp.split(self.scene_path)[1]})
        ET.SubElement(root, 'include', {'file': 'objects.xml'})
        for t in template_root:
            root.insert(-1, t)
        tree = ET.ElementTree(root)
        ET.indent(tree, space='\t', level=0)
        tree.write(self.scene_xml_file)
        print(f'{self.scene_xml_file} written')

        root = ET.Element('mujoco')
        for object_cat in sorted(os.listdir(self.output_folder)):
            cat_dir = osp.join(self.output_folder, object_cat)
            if not osp.isdir(cat_dir):
              continue
            for mesh_object in sorted(os.listdir(cat_dir)):
                if mesh_object == ".DS_Store":
                    continue
                ET.SubElement(root, 'include', {'file': osp.join('output', object_cat, mesh_object, f'{mesh_object}.xml')})
        tree = ET.ElementTree(root)
        ET.indent(tree, space='\t', level=0)
        filename = osp.join(self.scene_path, 'objects.xml')
        tree.write(filename)
        print(f'{filename} written')
    
    @staticmethod
    def _decimation_lookup(poly_count):
        # Get the adaptive decimation ratio
        if poly_count < 10000:
            return 1.0
        elif poly_count < 50000:
            return 0.8
        elif poly_count < 100000:
            return 0.6
        elif poly_count < 500000:
            return 0.5
        elif poly_count < 1000000:
            return 0.2
        elif poly_count < 5000000:
            return 0.1
        elif poly_count < 10000000:
            return 0.01
        elif poly_count < 20000000:
            return 0.009
        elif poly_count < 30000000:
            return 0.008
        elif poly_count < 40000000:
            return 0.009
        elif poly_count < 50000000:
            return 0.005
        else:
            return 0.001
    
    @staticmethod
    def _get_pose(obj):
        obj.select_set(True)
        xyz = copy.deepcopy(obj.location)
        xyz = [xyz.x, xyz.y, xyz.z]
        quat = copy.deepcopy(obj.rotation_quaternion)
        quat = [quat.w, quat.x, quat.y, quat.z]
        obj.select_set(False)
        return xyz, quat
    
    @staticmethod
    def _get_pose_and_center(obj, scale_factor=np.ones(3)):
        xyz, quat = AssetProcessor._get_pose(obj)
        xyz = np.asarray(xyz) * scale_factor
        obj.select_set(True)
        obj.location = (0.0, 0.0, 0.0)
        obj.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
        obj.select_set(False)
        return xyz.tolist(), quat        

    def _process_assets(self):
        # Select relevant objects in the scene (i.e. discard collision meshes, animations and so on...)
        objects = [obj for obj in bpy.context.scene.collection.all_objects if (obj.parent is None)]
        
        decompose_convex_args = []
        assemble_args = []
        for obj in objects:
            object_name = obj.name.split('.')[0]
            # Articulated furniture
            obj_type = self.get_object_type(object_name)
            obj_dir = os.path.join(self.output_folder, obj_type, object_name)

            # read joints info to augment it
            joints_info = {}
            if self.object_has_joint[obj.name.split('.')[0]]:
                filename = os.path.join(self.output_folder, '..', 'ue_export', f'{object_name}_joints.json')
                with open(filename, 'r') as f:
                    joints_info = json.load(f)
                joints_info = {j['name']: j for j in joints_info}

            children = []
            nodes_info = {}
            # each element is (bpy object, directory_path, scale factor)
            q = deque([(obj, obj_dir, np.ones(3)), ])
            while len(q):
                o, dir, scale_factor = q.popleft()
                name = o.name.split('.')[0]
                xyz, quat = self._get_pose_and_center(o, scale_factor)
                if o.children == ():
                    if (o.type == 'MESH') and (o.data.name != 'SM_Dummy'):
                        if name in nodes_info:
                            raise AssertionError(f'repeated mesh {name}')
                        nodes_info[name] = {'pos': xyz, 'quat': quat}
                        os.makedirs(dir, exist_ok=True)
                        children.append((dir, o, xyz, quat))
                    elif o.type == 'EMPTY':
                        joint_name = name_in_names(name, joints_info.keys())
                        if joint_name is not None:
                            joints_info[joint_name]['pos'], joints_info[joint_name]['quat'] = xyz, quat
                else:
                    if name in nodes_info:
                        raise AssertionError(f'repeated mesh {name}')
                    nodes_info[name] = {'pos': xyz, 'quat': quat}
                    os.makedirs(dir, exist_ok=True)
                    for child in o.children:
                        child_name = child.name.split('.')[0]
                        q.append((child, os.path.join(dir, child_name), np.array(o.scale)))

            # convert parent_T_joint to child_T_joint (which is what MuJoCo understands)
            for joint_info in joints_info.values():
                def getT(xyz, quat):
                    T = np.eye(4)
                    T[:3, :3] = txq.quat2mat(quat)
                    T[:3,  3] = xyz
                    return T
                pTj = getT(joint_info['pos'], joint_info['quat'])
                pTc = getT(nodes_info[joint_info['child']]['pos'], nodes_info[joint_info['child']]['quat'])
                cTj = np.linalg.inv(pTc) @ pTj
                joint_info['pos']  = cTj[:3, 3].tolist()
                joint_info['quat'] = txq.mat2quat(cTj[:3, :3]).tolist()
            
            joint_children = [j['child'] for j in joints_info.values()]
            for (child_dir, child, xyz, quat) in children:
                child_name = child.name.split('.')[0]
                child.select_set(True)
                
                if child_name in joint_children:
                    xyz = [0.0, 0.0, 0.0, 0.0]
                    quat = [1.0, 0.0, 0.0, 0.0]
                
                # Calculate the number of faces in the mesh
                poly_count = len(child.data.polygons)
                print("Polygon count: ", poly_count)
                
                # Get the adaptive decimation ratio
                decimation_ratio = AssetProcessor._decimation_lookup(poly_count)
                    
                # Add the Decimate modifier to the selected object
                print("Applying decimation with a ratio: ", decimation_ratio)
                decimate_modifier = child.modifiers.new(name='Decimate', type='DECIMATE')
                decimate_modifier.ratio = decimation_ratio

                # Exporting in stl.
                print("Exporting obj file...")
                part_filepath = os.path.join(child_dir, f"{child_name}.stl")
                bpy.ops.export_mesh.stl(
                    filepath=part_filepath,
                    check_existing=True,
                    use_selection=True,
                    use_mesh_modifiers=True,
                    batch_mode='OFF',
                    global_scale=1.0,
                    axis_forward='Y',
                    axis_up='Z',
                    ascii=False,
                )
                bpy.ops.object.select_all(action='DESELECT')

                print("Convex solid decomposition...")
                cvx_args = ACD_Args(use_CoACD, CoacdArgs(), VhacdArgs())
                decompose_convex_args.append(
                    (part_filepath, self.output_folder, self.scene_path, "cvx", cvx_args, xyz, quat, False, self.rerun)
                )
            assemble_args.append(
                (object_name, obj_dir, self.scene_path, nodes_info, joints_info,
                 self.actors_information[object_name]['moving'],
                 f'.{self.actors_information[object_name]["root_component_name"]}'),
            )
        # with mp.Pool(self.n_workers) as p:
        #     p.map(decompose_convex, decompose_convex_args)
        # with mp.Pool(self.n_workers) as p:
        #     p.map(assemble_articulated_object_files, assemble_args)
        i = 0
        while True:
            start_idx = i * self.n_workers
            end_idx   = (i+1) * self.n_workers
            with mp.Pool(self.n_workers) as p:
                p.map(decompose_convex, decompose_convex_args[start_idx : end_idx])
            i = i + 1
            if end_idx >= len(decompose_convex_args):
                break
        i = 0
        while True:
            start_idx = i * self.n_workers
            end_idx   = (i+1) * self.n_workers
            with mp.Pool(self.n_workers) as p:
                p.map(assemble_articulated_object_files, assemble_args[start_idx : end_idx])
            i = i + 1
            if end_idx >= len(assemble_args):
                break
    
    def process(self):
        # Deselect all objects
        bpy.ops.object.select_all(action='DESELECT')
        
        # Create a directory that will contain the decomposed assets
        os.makedirs(self.output_folder, exist_ok=True)
        
        # Asset convex decomposition
        # articulated objects should be processed first
        self._process_assets()

        # Generate a MuJoCo xml file
        self.generate_mujoco_scene()
                

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene_path', required=True, help='relative path to the scene folder')
    parser.add_argument('--rerun', action='store_true', help='re-run the export without running decomposition')
    parser.add_argument('--objects', help='comma separated', default=None)
    parser.add_argument('-n', type=int, help='number of parallel workers to use', default=mp.cpu_count()-1)
    args = parser.parse_args()
    proc = AssetProcessor(osp.expanduser(args.scene_path), args.n, args.rerun, args.objects)
    proc.process()
