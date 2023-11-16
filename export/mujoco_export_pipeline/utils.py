import coacd
from collections import deque
from itertools import combinations
import mujoco
import numpy as np
import os
import params
import subprocess
import shutil
import transforms3d.quaternions as txq
import trimesh
import xml.etree.ElementTree as ET


osp = os.path


def name_in_names(name, names):
  for n in names:
      if name in n:
          return n
  return None


def xyzquat_to_T(xyz, quat):
    T = np.eye(4)
    T[:3, :3] = txq.quat2mat(quat)
    T[:3,  3] = xyz
    return T


def T_to_xyzquat(T):
    return T[:3, 3].tolist(), txq.mat2quat(T[:3, :3]).tolist()


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
    object_path = object_folder.replace(scene_path, '')
    if object_path[0] == osp.sep:
        object_path = object_path[1:]
    
    _, _, mesh_objects = next(os.walk(osp.join(object_folder, cvx_folder)))
    for mesh_object in sorted(mesh_objects):
        split_file_name = osp.basename(mesh_object)
        split_name, ext = osp.splitext(split_file_name)
        if (ext != '.stl') and (ext != '.obj'):
            continue
        ET.SubElement(root, 'mesh',
                      {'file': osp.join(object_path, cvx_folder, split_file_name), 'name': f'{prefix}.{split_name}'}
        )
    
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
            if (file_name[1] != '.stl') and (file_name[1] != '.obj'):
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
            if ('.stl' not in file_name[1]) and ('.obj' not in file_name[1]):
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


def check_mesh_validity(filename: str) -> bool:
    """
    filename: STL file name
    """
    root = ET.Element('mujoco')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'mesh', {'file': filename, 'name': 'test_mesh_asset'})
    wbody = ET.SubElement(root, 'worldbody')
    body = ET.SubElement(wbody, 'body', {'name': 'test_body'})
    ET.SubElement(body, 'geom', {'name': 'test_mesh', 'mesh': 'test_mesh_asset'})
    try:
        m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        d = mujoco.MjData(m)
    except:
        print(f'!!!!!!!! invalid mesh {filename} !!!!!!!!')
        return False
    return True


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
        
        if check_mesh_validity(obj_filename):
            mesh: trimesh.Trimesh = trimesh.load(obj_filename, force='mesh')
            if mesh.is_empty:
                volume = 0.0
            else:
                try:
                    volume = mesh.bounding_box_oriented.volume
                except ValueError:
                    volume = 0.0
            if volume < 1e-6:
                acd_args.decompose_method = 'skip'
                print(f'{obj_name} is too small, volume = {volume:.5f}, will skip')
            elif volume < acd_args.min_volume:
                acd_args.decompose_method = 'none'
                print(f'{obj_name} is small, volume = {volume:.5f}, will not decompose')
            
            if acd_args.decompose_method == 'coacd':
                # Call CoACD.
                if acd_args.coacd_args.auto_threshold:
                    model = acd_args.coacd_args.threshold_model
                    threshold = max(0.02, volume*model[0] + model[1])
                else:
                    threshold = acd_args.coacd_args.threshold
                mesh = coacd.Mesh(mesh.vertices, mesh.faces)
                result = coacd.run_coacd(
                    mesh,
                    threshold=threshold,
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
                    if mesh.is_volume:
                        mesh_parts.append(mesh)

                object_assembled = trimesh.Scene()
                np.random.seed(0)
                i = 0
                for p in mesh_parts:
                    p.visual.vertex_colors[:, :3] = (np.random.rand(3) * 255).astype(np.uint8)
                    object_part = trimesh.Scene()
                    object_part.add_geometry(p)
                    filename = osp.join(cvx_path, f'{obj_name}{i:03d}.stl')
                    object_part.export(filename)
                    if check_mesh_validity(filename):
                        object_assembled.add_geometry(p)
                        i+=1
                if not object_assembled.is_empty:
                    object_assembled.export(osp.join(obj_dir, f'{obj_name}_{cvx_dir}.obj'))
                    decomposed = True
            elif acd_args.decompose_method == 'vhacd':
                if not acd_args.vhacd_args.enable:
                    return False

                if params.VHACD_EXECUTABLE is None:
                    print(
                        "V-HACD was enabled but not found in the system path. Either install it "
                        "manually or run `bash install_vhacd.sh`. Skipping decomposition"
                    )
                    return False

                # Call V-HACD, suppressing output.
                ret = subprocess.run(
                    [
                        f"{params.VHACD_EXECUTABLE}",
                        osp.join(cvx_dir, obj_file),
                        "-i",
                        "stl",
                        "-o",
                        "stl",
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
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
                output_str = ret.stdout.decode()
                print(output_str)
                        
                if (ret.returncode != 0) or ('VolumeError' in output_str):
                    print(f"V-HACD failed on {fullpath}")
                else: 
                    decomposed = True
                # VHACD not being able to generate a convex hull decoposition (while not throwing any error) is a
                # good indicator of the level of fucked-upness of an asset. For now, we simply discard the asset.
                os.rename(osp.join(obj_dir, params.VHACD_OUTPUTS[0]), osp.join(obj_dir, f'{obj_name}_{cvx_dir}.obj'))
                os.remove(osp.join(obj_dir, params.VHACD_OUTPUTS[1]))
            elif acd_args.decompose_method == 'none':
                # simply copy the STL
                out_filename = osp.join(obj_dir, f'{obj_name}_{cvx_dir}.obj')
                mesh.export(out_filename)
                shutil.copy(obj_filename, osp.join(cvx_path, f'{obj_name}000.stl'))
                decomposed = True
            elif acd_args.decompose_method == 'skip':
                decomposed = False
            else:
                raise ValueError(f'{obj_dir} decompose method {acd_args.decompose_method} is invalid')
        
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

    # elements are (directory, parent directoy, parent body tag, name prefix)
    q = deque([(obj_dir, None, worldbody, ''), ])
    sibling_dir_groups = []
    dir2bodyname = {}
    leaf_dirs = set()
    while len(q):
        dir, parent_dir, parent, name_prefix = q.popleft()
        parent_name = osp.split(parent_dir)[-1] if parent_dir is not None else None
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
                    'range': '{:.5f} {:.5f}'.format(*joint[0]['range']),
                    'ref': '{:.5f}'.format(joint[0]['ref'])
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
        q.extendleft([(child_dir, dir, geom_parent, name_prefix) for child_dir in child_dirs])
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