import argparse
from collections import deque
from itertools import combinations
import json
import mujoco
import numpy as np
import os
import params
import utils
import xml.etree.ElementTree as ET
import yaml


osp = os.path


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


class MuJoCoSceneAssembler(object):
    def __init__(self, scene_path: str, include_objects: str) -> None:
        self.scene_name = osp.split(scene_path)[1]
        self.input_dir = osp.join(scene_path, params.COLLISION_DIR_NAME)
        self.output_dir = osp.join(scene_path, params.PHYSICS_DIR_NAME.format('mujoco'))
        if include_objects is not None:
            include_objects = include_objects.split(',')
        self.include_objects = include_objects
        if not osp.isdir(self.input_dir):
            raise FileNotFoundError(f'Missing collision representation at {self.input_dir}')
        # TODO(samarth): 1 yaml file per pipeline stage, which are in the export_pipeline dir
        filename = osp.join(scene_path, 'mujoco_params.yaml')
        with open(filename, 'r') as f:
            self.mujoco_params = yaml.safe_load(f)


    def run(self):
        filename = osp.join(self.input_dir, '..', params.UE_EXPORT_DIR_NAME, 'actors_information.json')
        #TODO(samarth) BFS
        with open(filename, 'r') as f:
            actors_information = json.load(f)
        for object_name in sorted(next(os.walk(self.input_dir))[1]):
            obj_dir = osp.join(self.input_dir, object_name)
            filename = osp.join(obj_dir, 'nodes.json')
            with open(filename, 'r') as f:
                nodes_info = json.load(f)
            for mesh_dir in sorted(next(os.walk(obj_dir))[1]):
                xyz  = nodes_info[mesh_dir]['pos']
                quat = nodes_info[mesh_dir]['quat']
                mesh_dir = osp.join(obj_dir, mesh_dir)
                self.assemble_mesh(mesh_dir, xyz, quat)
            filename = osp.join(obj_dir, 'joints.json')
            try:
                with open(filename, 'r') as f:
                    joints_info = json.load(f)
                self.assemble_object_with_joints(obj_dir, nodes_info, joints_info, actors_information[object_name])
            except FileNotFoundError:
                continue
        self.generate_scene()
    
    
    def assemble_mesh(self, mesh_dir, xyz, quat):
        """
        Assembles the convex regions of a mesh
        """
        mesh_name = osp.split(mesh_dir)[1]
        path_prefix = mesh_dir.replace(self.input_dir, '')
        if path_prefix[0] == osp.sep:
            path_prefix = path_prefix[1:]
        output_dir = osp.join(self.output_dir, path_prefix)
        os.makedirs(output_dir, exist_ok=True)

        # write assets file
        root = ET.Element('mujocoinclude', {'model': mesh_name})
        asset_name_prefix = path_prefix.replace(osp.sep, '.')
        asset_path_prefix = osp.join('..', params.COLLISION_DIR_NAME, path_prefix, params.CONVEX_DECOMPOSITION_DIR)
        asset_names = []
        for mesh_filename in sorted(next(os.walk(osp.join(mesh_dir, params.CONVEX_DECOMPOSITION_DIR)))[2]):
            region_name, ext = osp.splitext(osp.basename(mesh_filename))
            if (ext != params.CONVEX_DECOMPOSITION_EXT) or (region_name == 'assembled'):
                continue
            asset_name = f'{asset_name_prefix}.{region_name}'
            asset_names.append(asset_name)
            asset_attributes = {
                'file': osp.join(asset_path_prefix, mesh_filename),
                'name': asset_name
            }
            ET.SubElement(root, 'mesh', asset_attributes)
        filename = osp.join(output_dir, 'assets.xml')
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)
        tree.write(filename)
        print(f'{filename} written')

        # write body file
        root = ET.Element('mujocoinclude', {'model': mesh_name})
        for asset_name in asset_names:
            geom_attributes = {
                'name': f'{asset_name}_vis',
                'mesh': asset_name,
                'class': 'visual',
                'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
            }
            ET.SubElement(root, 'geom', geom_attributes)
            geom_attributes = {
                'name': f'{asset_name}_col',
                'mesh': asset_name,
                'class': 'collision',
                'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
            }
            ET.SubElement(root, 'geom', geom_attributes)
        filename = osp.join(output_dir, 'body.xml')
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)
        tree.write(filename)
        print(f'{filename} written')
        

    def assemble_object_with_joints(self, obj_dir, nodes_info, joints_info, obj_info):
        obj_name = osp.split(obj_dir)[1]
        parent_name_suffix = f'.{obj_info["root_component_name"]}'
        full_obj_name = f'{obj_name}{parent_name_suffix}'
        root = ET.Element('mujocoinclude', {'model': obj_name})
        asset = ET.SubElement(root, 'asset')
        worldbody = ET.SubElement(root, 'worldbody')
        contact = ET.SubElement(root, 'contact')
        actuator = ET.SubElement(root, 'actuator')

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
                pos, quat = utils.pose_rh_to_lh(nodes_info[name]['pos'], nodes_info[name]['quat'])
                body_attributes = {
                    'name': body_name,
                    'pos': '{:f} {:f} {:f}'.format(*pos),
                    'quat': '{:f} {:f} {:f} {:f}'.format(*quat)
                }
                body = ET.SubElement(parent, 'body', body_attributes)
                if len(joint) == 1:
                    joint_name = f'{name_prefix}{joint[0]["name"]}{parent_name_suffix}'
                    pos, _ = utils.pose_rh_to_lh(joint[0]['pos'], (1, 0, 0, 0))
                    axis, _ = utils.pose_rh_to_lh(joint[0]['axis'], (1, 0, 0, 0))
                    joint_attributes = {
                        'name': joint_name,
                        'type': joint[0]['type'],
                        'pos': '{:.5f} {:.5f} {:.5f}'.format(*pos),
                        'axis': '{:.5f} {:.5f} {:.5f}'.format(*axis),
                        'range': '{:.5f} {:.5f}'.format(*joint[0]['range']),
                        'ref': '{:.5f}'.format(joint[0]['ref']),
                        'damping': f'{self.mujoco_params["joint_kd"]:.5f}',
                    }
                    ET.SubElement(body, 'joint', joint_attributes)
                    actuator_attributes = {
                        'kp': f'{self.mujoco_params["joint_kp"]:.5f}',
                        'ctrllimited': 'true',
                        'ctrlrange': '{:.5f} {:.5f}'.format(*np.deg2rad(joint[0]['range'])),
                        'joint': joint_name,
                        'name': f'act:{joint_name}',
                    }
                    ET.SubElement(actuator, 'position', actuator_attributes)
                elif obj_info['moving']:
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
            if osp.isdir(osp.join(dir, params.CONVEX_DECOMPOSITION_DIR)):
                path_prefix = dir.replace(self.input_dir, '')
                if path_prefix[0] == osp.sep:
                    path_prefix = path_prefix[1:]
                # path_prefix = osp.join('..', params.COLLISION_DIR_NAME, path_prefix)
                ET.SubElement(geom_parent, 'include', {'file': osp.join(path_prefix, 'body.xml')})
                ET.SubElement(asset, 'include', {'file': osp.join(path_prefix, 'assets.xml')})
            
            # explore children
            _, child_dirs, _ = next(os.walk(dir))
            child_dirs = [osp.join(dir, child_dir) for child_dir in sorted(child_dirs) if child_dir != params.CONVEX_DECOMPOSITION_DIR]
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
        filename = osp.join(obj_dir, f'{obj_name}.xml')
        filename = filename.replace(self.input_dir, self.output_dir)
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)
        tree.write(filename)
        print(f'{filename} written')


    def generate_scene(self):
        template_root = ET.parse(osp.join('export_pipeline', 'scene_include.xml')).getroot()
        root = ET.Element('mujoco', {'model': self.scene_name})
        ET.SubElement(root, 'include', {'file': 'objects.xml'})
        for t in template_root:
            root.insert(-1, t)
        tree = ET.ElementTree(root)
        ET.indent(tree, space='\t', level=0)
        filename = osp.join(self.output_dir, "scene.xml")
        tree.write(filename)
        print(f'{filename} written')

        root = ET.Element('mujoco')
        for body_name in sorted(next(os.walk(self.output_dir))[1]):
            if body_name == ".DS_Store":
                continue
            ET.SubElement(root, 'include', {'file': osp.join(body_name, f'{body_name}.xml')})
        tree = ET.ElementTree(root)
        ET.indent(tree, space='\t', level=0)
        filename = osp.join(self.output_dir, 'objects.xml')
        tree.write(filename)
        print(f'{filename} written')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene_path', required=True, help='relative path to the scene folder')
    parser.add_argument('--objects', help='comma separated', default=None)
    args = parser.parse_args()
    
    mse = MuJoCoSceneAssembler(osp.expanduser(args.scene_path), args.objects)
    mse.run()