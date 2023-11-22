import argparse
from collections import deque, defaultdict
import copy
import json
import multiprocessing as mp
if mp.current_process().name == 'MainProcess':
    import bpy
import numpy as np
import os
import params
import shutil
import transforms3d.quaternions as txq
import utils
import xml.etree.ElementTree as ET


osp = os.path


class MujocoExporter:
    def __init__(self, scene_path: str, n_workers: int, rerun: bool, include_objects: str) -> None:
        self.n_workers = min(max(1, n_workers), mp.cpu_count()-1)
        self.rerun = rerun
        ue_export_path = osp.join(scene_path, 'ue_export')
        self.input_gltf_filenames = {}
        for filename in next(os.walk(ue_export_path))[2]:
            if not filename.endswith('.gltf'):
                continue
            if include_objects and not any([(osp.splitext(filename)[0] == i) for i in include_objects.split(',')]):
                continue
            joint_filename = filename.replace('.gltf', '_joints.json')
            self.input_gltf_filenames[osp.join(ue_export_path, filename)] = osp.isfile(osp.join(ue_export_path, joint_filename))
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
        template_root = ET.parse(osp.join('mujoco_export_pipeline', 'scene_include.xml')).getroot()
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
    def _get_relative_pose(obj):
        """
        gets pose relative to parent
        """
        obj.select_set(True)
        xyz = copy.deepcopy(obj.location)
        xyz = [xyz.x, xyz.y, xyz.z]
        quat = copy.deepcopy(obj.rotation_quaternion)
        quat = [quat.w, quat.x, quat.y, quat.z]
        obj.select_set(False)
        return xyz, quat
    
    
    @staticmethod
    def _get_relative_pose_and_center(obj, scale_factor=np.ones(3)):
        """
        gets pose relative to parent, and centers obj w.r.t. parent
        """
        xyz, quat = MujocoExporter._get_relative_pose(obj)
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
            obj_info = self.actors_information[object_name]

            # read joints info to augment it
            joints_info = {}
            if self.object_has_joint[obj.name.split('.')[0]]:
                filename = os.path.join(self.output_folder, '..', 'ue_export', f'{object_name}_joints.json')
                with open(filename, 'r') as f:
                    joints_info = json.load(f)
                joints_info = {j['name']: j for j in joints_info}

            # traverse the tree, whose leaves will be meshes. Collect leaves and their info for decomposition.
            leaves = []
            nodes_info = {}
            # each element is (bpy object, directory_path, parent transform, scale_factor, group_id, is_root)
            # parent transform is reset to I at every joint. It is also I for the actor root
            q = deque([(obj, obj_dir, np.eye(4), np.ones(3), object_name, True), ])
            while len(q):
                o, dir, T_parent, scale_factor, group_id, is_root = q.popleft()
                name = o.name.split('.')[0]
                xyz, quat = self._get_relative_pose_and_center(o, scale_factor)
                T_o = T_parent @ utils.xyzquat_to_T(xyz, quat)
                xyz, quat = utils.T_to_xyzquat(T_o)
                if o.children == ():  # StaticMeshComponent with geometry, or PhysicsConstraintComponent (i.e. joint)
                    if (o.type == 'MESH') and (o.data.name != 'SM_Dummy'):  # StaticMeshComponent with geometry
                        if name in nodes_info:
                            raise AssertionError(f'repeated mesh {name}')
                        nodes_info[name] = {'pos': xyz, 'quat': quat}
                        os.makedirs(dir, exist_ok=True)
                        decompose_method = obj_info['geoms'][name]['decompose_method']
                        leaves.append((dir, o, xyz, quat, decompose_method, group_id))
                    elif o.type == 'EMPTY':  # PhysicsConstraintComponent (i.e. joint)
                        joint_name = utils.name_in_names(name, joints_info.keys())
                        if joint_name is not None:
                            joints_info[joint_name]['pos'], joints_info[joint_name]['quat'] = xyz, quat
                else:  # StaticMeshComponent without any geometry (meant for grouping)
                    if name in nodes_info:
                        raise AssertionError(f'repeated mesh {name}')
                    nodes_info[name] = {'pos': xyz, 'quat': quat}
                    os.makedirs(dir, exist_ok=True)
                    if is_root or any([j['child'] == name for j in joints_info.values()]):
                        T_parent = np.eye(4)
                    else:
                        T_parent = np.copy(T_o)
                    for child in o.children:
                        child_name = child.name.split('.')[0]
                        if child_name in obj_info['geoms']:
                            group_id = obj_info['geoms'][child_name]['decompose_group']
                            group_id = f'{name}_{group_id}'
                        else:
                            group_id = None
                        q.append(
                            (child, osp.join(dir, child_name), T_parent, scale_factor*np.array(o.scale), group_id, False)
                        )

            # convert parent_T_joint to child_T_joint (which is what MuJoCo understands)
            for joint_info in joints_info.values():
                pTj = utils.xyzquat_to_T(joint_info['pos'], joint_info['quat'])
                pTc = utils.xyzquat_to_T(nodes_info[joint_info['child']]['pos'], nodes_info[joint_info['child']]['quat'])
                cTj = np.linalg.inv(pTc) @ pTj
                joint_info['pos']  = cTj[:3, 3].tolist()
                joint_info['quat'] = txq.mat2quat(cTj[:3, :3]).tolist()

            # group the children by group_id
            geom_groups = defaultdict(list)
            for leaf in leaves:
                geom_groups[leaf[-1]].append(leaf[:-1])
            
            joint_children = [j['child'] for j in joints_info.values()]
            for geom_group in geom_groups.values():
                leaf_dir, leaf, xyz, quat, decompose_method = geom_group[0]
                leaf_name = leaf.name.split('.')[0]
                
                # merge the geoms
                for l in geom_group[1:]:
                    l[1].select_set(True)
                    shutil.rmtree(l[0])
                leaf.select_set(True)
                bpy.context.view_layer.objects.active = leaf
                bpy.ops.object.join()
                
                if leaf_name in joint_children:
                    # zero out the transform w.r.t. parent, because this mesh's include file will be placed under
                    # an appropriately transformed body
                    xyz = [0.0, 0.0, 0.0, 0.0]
                    quat = [1.0, 0.0, 0.0, 0.0]
                
                # Calculate the number of faces in the mesh
                poly_count = len(leaf.data.polygons)
                print("Polygon count: ", poly_count)
                
                # Get the adaptive decimation ratio
                decimation_ratio = MujocoExporter._decimation_lookup(poly_count)
                    
                # Add the Decimate modifier to the selected object
                print("Applying decimation with a ratio: ", decimation_ratio)
                decimate_modifier = leaf.modifiers.new(name='Decimate', type='DECIMATE')
                decimate_modifier.ratio = decimation_ratio

                # Exporting in stl.
                stl_filepath = osp.join(leaf_dir, f"{leaf_name}.stl")
                if (not self.rerun) or (not osp.isfile(stl_filepath)):
                    print("Exporting STL file...")
                    bpy.ops.export_mesh.stl(
                        filepath=stl_filepath,
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

                cvx_args = params.ACD_Args(decompose_method, params.CoacdArgs(), params.VhacdArgs())
                decompose_convex_args.append(
                    (stl_filepath, self.output_folder, self.scene_path, "cvx", cvx_args, xyz, quat, False, self.rerun)
                )
            assemble_args.append(
                (object_name, obj_dir, self.scene_path, nodes_info, joints_info,
                 obj_info['moving'],
                 f'.{obj_info["root_component_name"]}'),
            )
        # with mp.Pool(self.n_workers) as p:
        #     p.map(decompose_convex, decompose_convex_args)
        # with mp.Pool(self.n_workers) as p:
        #     p.map(assemble_articulated_object_files, assemble_args)
        print("Convex solid decomposition...")
        i = 0
        while True:
            start_idx = i * self.n_workers
            end_idx   = (i+1) * self.n_workers
            with mp.Pool(self.n_workers) as p:
                p.map(utils.decompose_convex, decompose_convex_args[start_idx : end_idx])
            i = i + 1
            if end_idx >= len(decompose_convex_args):
                break
        print("Assembly...")
        i = 0
        while True:
            start_idx = i * self.n_workers
            end_idx   = (i+1) * self.n_workers
            with mp.Pool(self.n_workers) as p:
                p.map(utils.assemble_articulated_object_files, assemble_args[start_idx : end_idx])
            i = i + 1
            if end_idx >= len(assemble_args):
                break
    
    
    def export(self):
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
    
    exporter = MujocoExporter(osp.expanduser(args.scene_path), args.n, args.rerun, args.objects)
    exporter.export()