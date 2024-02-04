"""
This script decomposes all static meshes into sets of convex regions.
Decomposition method and parameters are based on the parameters chosen in UE, which are captured in 
gltf_scene/actors_information.json.
"""
import argparse
from collections import deque, defaultdict
import copy
import json
import multiprocessing as mp
if mp.current_process().name == 'MainProcess':
  import bpy
import coacd  # needs to be imported after bpy, otherwise terminate called after throwing an instance of 'std::bad_cast'
import numpy as np
import os
import shutil
import subprocess
import trimesh
import utils
import yaml


osp = os.path


def decimation_lookup(poly_count):
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


def get_relative_pose(component):
    """
    gets pose relative to parent
    """
    component.select_set(True)
    xyz = copy.deepcopy(component.location)
    xyz = [xyz.x, xyz.y, xyz.z]
    quat = copy.deepcopy(component.rotation_quaternion)
    quat = [quat.w, quat.x, quat.y, quat.z]
    component.select_set(False)
    return xyz, quat


def get_relative_pose_and_center(component, scale_factor=np.ones(3)):
    """
    gets pose relative to parent, and centers obj w.r.t. parent
    """
    xyz, quat = get_relative_pose(component)
    xyz = np.asarray(xyz) * scale_factor
    component.select_set(True)
    component.location = (0.0, 0.0, 0.0)
    component.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
    component.select_set(False)
    return xyz.tolist(), quat


def process_mesh(args: tuple) -> bool:
    """
    Decompose a mesh into convex regions according to hyperparameters.
    Save the convex regions as well as a convex hull of the mesh in a directory structure.
    args: a tuple containing (path to mesh, hyperparameters, rerun flag).
    Returns a bool indicating success of the operation.
    """
    mesh_path, params, rerun = args
    mesh_dir, mesh_filename = osp.split(mesh_path)
    vhacd_args = params['collision_representation']['vhacd']
    coacd_args = params['collision_representation']['coacd']
    acd_args   = params['collision_representation']['acd']

    decomposed = False
    decompose_dir = osp.join(mesh_dir, acd_args['decompose_method'])
    if rerun and osp.isdir(decompose_dir):
        decomposed = True
        os.remove(mesh_path)
    else:
        os.makedirs(decompose_dir, exist_ok=True)
        mesh_path = shutil.move(mesh_path, decompose_dir)
        mesh_name = os.path.splitext(mesh_filename)[0]
        
        # load mesh and check that it is valid
        mesh: trimesh.Trimesh = trimesh.load(mesh_path, force='mesh')
        if mesh.is_empty:
            volume = 0.0
        else:
            try:
                volume = mesh.bounding_box_oriented.volume
            except ValueError:
                volume = 0.0
        if volume < 1e-6:
            acd_args['decompose_method'] = 'skip'
            print(f'{mesh_name} is too small, volume = {volume:.5f}, will skip')
        elif volume < acd_args['min_volume']:
            acd_args['decompose_method'] = 'none'
            print(f'{mesh_name} is small, volume = {volume:.5f}, will not decompose')

        # save convex hull
        hull_mesh: trimesh.Trimesh = trimesh.convex.convex_hull(mesh)
        hull_filename = osp.join(mesh_dir, 'convex_hull.stl')
        hull_mesh.export(hull_filename)
        print(f'Convex hull {hull_filename} saved.')
        
        if acd_args['decompose_method'] == 'coacd':
            # Call CoACD.
            if coacd_args['auto_threshold']:
                model = coacd_args['threshold_model']
                threshold = max(0.02, volume*model[0] + model[1])
            else:
                threshold = coacd_args['threshold']
            mesh = coacd.Mesh(mesh.vertices, mesh.faces)
            result = coacd.run_coacd(
                mesh,
                threshold=threshold,
                max_convex_hull=coacd_args['max_convex_hull'],
                preprocess_mode=coacd_args['preprocess_mode'],
                preprocess_resolution=coacd_args['preprocess_resolution'],
                resolution=coacd_args['resolution'],
                mcts_nodes=coacd_args['mcts_nodes'],
                mcts_iterations=coacd_args['mcts_iterations'],
                mcts_max_depth=coacd_args['mcts_max_depth'],
                pca=coacd_args['pca'],
                merge=coacd_args['merge'],
                seed=coacd_args['seed'],
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
                filename = osp.join(decompose_dir, f"{i:03d}{params['common']['CONVEX_DECOMPOSITION_EXT']}")
                object_part.export(filename)
                object_assembled.add_geometry(p)
                i+=1
            if not object_assembled.is_empty:
                object_assembled.export(osp.join(decompose_dir, 'assembled.obj'))
                decomposed = True
        elif acd_args['decompose_method'] == 'vhacd':
            if not vhacd_args['enable']:
                return False

            if shutil.which('TestVHACD') is None:
                print(
                    "V-HACD was enabled but not found in the system path. Either install it "
                    "manually or run `bash install_vhacd.sh`. Skipping decomposition"
                )
                return False

            # Call V-HACD, suppressing output.
            fill_mode = {0: 'floor', 1: 'surface', 2: 'raycast'}
            ret = subprocess.run(
                [
                    shutil.which('TestVHACD'),
                    osp.join('convex_decomposition', mesh_filename),
                    "-i",
                    "stl",
                    "-o",
                    params['common']['CONVEX_DECOMPOSITION_EXT'][1:],
                    "-h",
                    f"{vhacd_args['max_output_convex_hulls']}",
                    "-r",
                    f"{vhacd_args['voxel_resolution']}",
                    "-e",
                    f"{vhacd_args['volume_error_percent']}",
                    "-d",
                    f"{vhacd_args['max_recursion_depth']}",
                    "-s",
                    f"{int(not vhacd_args['disable_shrink_wrap'])}",
                    "-f",
                    f"{vhacd_args['fill_mode'].name.lower()}",
                    "-v",
                    f"{vhacd_args['max_hull_vert_count']}",
                    "-a",
                    f"{int(not vhacd_args['disable_asyn'])}",
                    "-l",
                    f"{vhacd_args['min_edge_length']}",
                    "-p",
                    f"{int(vhacd_args['split_hull'])}",
                ],
                check=True,
                cwd=mesh_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            output_str = ret.stdout.decode()
            print(output_str)
                    
            if (ret.returncode != 0) or ('VolumeError' in output_str):
                print(f"V-HACD failed on {mesh_path}")
            else: 
                decomposed = True
            shutil.move(osp.join(mesh_dir, vhacd_args['outputs'][1]), osp.join(decompose_dir, 'assembled.stl'))
            os.remove(osp.join(mesh_dir, vhacd_args['outputs'][0]))
            os.remove(osp.join(mesh_dir, vhacd_args['outputs'][2]))
            for src_filename in next(os.walk(decompose_dir))[-1]:
                dst_filename = src_filename.replace(mesh_name, '')
                if dst_filename == '.stl':
                    continue
                shutil.move(osp.join(decompose_dir, src_filename), osp.join(decompose_dir, dst_filename))
        elif acd_args['decompose_method'] == 'none':
            out_filename = osp.join(decompose_dir, 'assembled.stl')
            mesh.export(out_filename)
            out_filename = osp.join(decompose_dir, '000.stl')
            if params['common']['CONVEX_DECOMPOSITION_EXT'] in mesh_path:
                shutil.copy(mesh_path, out_filename)
            else:
                mesh.export(out_filename)
            decomposed = True
        elif acd_args['decompose_method'] == 'skip':
            decomposed = False
            shutil.rmtree(decompose_dir)
        else:
            raise ValueError(f'{mesh_dir} decompose method {acd_args["decompose_method"]} is invalid')
        
        print(f"Removing {mesh_path}")
        os.remove(mesh_path)
        
    if not decomposed:  # delete whole directory
        shutil.rmtree(mesh_dir)
    return decomposed


class CollisionRepresentationComputer(object):
    def __init__(self, scene_path: str, n_workers: int, rerun: bool, include_objects: str) -> None:
        self.n_workers = min(max(1, n_workers), mp.cpu_count()-1)
        self.rerun = rerun
        self.scene_path = osp.normpath(scene_path)
        if include_objects is not None:
            include_objects = include_objects.split(',')
        self.input_gltf_filenames = []
        with open(osp.join('export_pipeline', 'params.yaml'), 'r') as f:
            self.p = yaml.safe_load(f)
        self.input_dir = osp.join(scene_path, self.p['common']['GLTF_SCENE_DIR_NAME'])
        self.output_dir = osp.join(scene_path, self.p['common']['COLLISION_DIR_NAME'])
        for filename in next(os.walk(self.input_dir))[2]:
            if not filename.endswith('.gltf'):
                continue
            if include_objects and not any([osp.splitext(filename)[0] == i for i in include_objects]):
                continue
            self.input_gltf_filenames.append(osp.join(self.input_dir, filename))


    def run(self):
        if not self.rerun:
            shutil.rmtree(self.output_dir, ignore_errors=True)

        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()
        
        with open(osp.join(self.input_dir, 'actors_information.json'), 'r') as f:
            actors_information = json.load(f)
        
        process_mesh_args = []
        for input_gltf_filename in self.input_gltf_filenames:
            bpy.ops.import_scene.gltf(filepath=input_gltf_filename)
            actor = bpy.context.object  # recently imported object
            
            actor_name = actor.name.split('.')[0]
            actor_dir = os.path.join(self.output_dir, actor_name)
            actor_info = actors_information[actor_name]

            joint_filename = input_gltf_filename.replace('.gltf', '_joints.json')
            try:
                with open(joint_filename, 'r') as f:
                    joints_info = json.load(f)
                joints_info = {j['name']: j for j in joints_info}
            except FileNotFoundError:
                joints_info = {}
            KT_node_names = [actor_name, ]
            KT_node_names.extend([j['child'] for j in joints_info.values()])

            # BFS tree traversal and collecting component information
            geom_groups = defaultdict(list)
            decompose_methods = {}
            components_info = {'static': {}, 'joints': {}}
            # (component, component_parent_dir, component_scale_factor, node_T_parent)
            q = deque([(actor, self.output_dir, np.ones(3), np.eye(4))])
            while len(q):
                component, component_parent_dir, component_scale_factor, node_T_parent = q.popleft()
                parent_name = osp.split(component_parent_dir)[1]
                name = component.name.split('.')[0]
                xyz, quat = get_relative_pose_and_center(component, component_scale_factor)
                node_T_component = node_T_parent @ utils.xyzquat_to_T(xyz, quat)
                xyz, quat = utils.T_to_xyzquat(node_T_component)

                if name in KT_node_names:
                    component_parent_dir = osp.join(component_parent_dir, name)
                    parent_name = name
                    os.makedirs(component_parent_dir, exist_ok=True)
                    components_info['static'][name] = {'pos': xyz, 'quat': quat, 'meshes': {}}
                    node_T_o = np.eye(4)
                    xyz = [0.0, 0.0, 0.0]
                    quat = [1.0, 0.0, 0.0, 0.0]

                is_geom = (component.type == 'MESH') and (component.data.name != 'SM_Dummy')
                is_joint = (component.type == 'EMPTY') and (len(component.children) == 0)

                if is_geom:
                    merge_id = actor_info['geoms'][name]['merge_id']
                    decompose_dir = osp.join(component_parent_dir, self.p['common']['CONVEX_DECOMPOSITION_DIR'], merge_id)
                    decompose_method = actor_info['geoms'][name]['decompose_method']
                    if decompose_dir not in decompose_methods:
                        decompose_methods[decompose_dir] = decompose_method
                        components_info['static'][parent_name]['meshes'][merge_id] = {
                            'pos': xyz,
                            'quat': quat,
                            'decompose_method': decompose_method,
                        }
                    elif decompose_methods[decompose_dir] != decompose_method:
                        raise ValueError(f'{decompose_dir} is marked to be decomposed by {decompose_methods},'
                                         'which does not match with other meshes to be merged with it.')
                    geom_groups[decompose_dir].append((component, xyz, quat))

                if is_joint:
                    joint_name = utils.name_in_names(name, joints_info.keys())
                    components_info['joints'][joint_name] = {'pos':  xyz, 'quat': quat}

                for child_component in component.children:
                    q.append((child_component, component_parent_dir, component_scale_factor*component.scale, node_T_o))
            
            filename = osp.join(actor_dir, 'components.json')
            with open(filename, 'w') as f:
                json.dump(components_info, f, indent=4)

            for decompose_dir, geom_group in geom_groups.items():
                # merge all geoms in the geom group, after transforming them into the node's coordinate frame
                for (component, node_T_component_xyz, node_T_component_quat) in geom_group:
                    component.location = node_T_component_xyz
                    component.rotation_quaternion = node_T_component_quat
                    component.select_set(True)
                geom = geom_group[0][0]
                bpy.context.view_layer.objects.active = geom
                bpy.ops.object.join()
                geom.location = (0.0, 0.0, 0.0)
                geom.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)

                # Calculate the number of faces in the mesh
                poly_count = len(geom.data.polygons)
                
                # Get the adaptive decimation ratio
                decimation_ratio = decimation_lookup(poly_count)
                    
                # Add the Decimate modifier to the selected object
                print("Applying decimation with a ratio: ", decimation_ratio)
                decimate_modifier = geom.modifiers.new(name='Decimate', type='DECIMATE')
                decimate_modifier.ratio = decimation_ratio

                # Exporting in stl.
                os.makedirs(decompose_dir, exist_ok=True)
                stl_filepath = osp.join(decompose_dir, "geom.stl")
                if (not self.rerun) or (not osp.isfile(stl_filepath)):
                    print("Exporting STL file...")
                    bpy.ops.export_mesh.stl(
                        filepath=stl_filepath,
                        check_existing=False,
                        use_selection=True,
                        use_mesh_modifiers=True,
                        batch_mode='OFF',
                        global_scale=1.0,
                        axis_forward='Y',
                        axis_up='Z',
                        ascii=False,
                    )
                bpy.ops.object.select_all(action='DESELECT')

                params = copy.deepcopy(self.p)
                params['collision_representation']['acd']['decompose_method'] = decompose_methods[decompose_dir]
                process_mesh_args.append((stl_filepath, params, self.rerun))

        print("Convex solid decomposition...")
        i = 0
        while True:
            start_idx = i * self.n_workers
            end_idx   = (i+1) * self.n_workers
            with mp.Pool(self.n_workers) as p:
                p.map(process_mesh, process_mesh_args[start_idx : end_idx])
            i = i + 1
            if end_idx >= len(process_mesh_args):
                break


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene_path', required=True, help='relative path to the scene folder')
    parser.add_argument('--rerun', action='store_true', help='re-run the export without running decomposition')
    parser.add_argument('--objects', help='comma separated', default=None)
    parser.add_argument('-n', type=int, help='number of parallel workers to use', default=mp.cpu_count()-1)
    args = parser.parse_args()
    
    crc = CollisionRepresentationComputer(osp.expanduser(args.scene_path), args.n, args.rerun, args.objects)
    crc.run()