import argparse
import os

import mujoco
import numpy as np
from urdf_parser_py import urdf

import xml.etree.ElementTree as ET


def scale_mjcf_mesh(src, dst, scale):
    tree = ET.parse(src)
    root = tree.getroot()
    if root.tag == "mujoco":
        asset = root.find("asset")
        for mesh in asset.findall("mesh"):
            mesh.attrib["scale"] = f"{scale} {scale} {scale}"
    tree.write(dst)


def save_urdf_file_to_mjcf(src, dst):
    # load to mujoco
    mj_model = mujoco.MjModel.from_xml_path(src)
    mj_data = mujoco.MjData(mj_model)
    # mujoco.mj_forward(mj_model, mj_data)
    # export complete temp.mjcf
    mujoco.mj_saveLastXML(dst, mj_model)


mujoco_config_xml = """
<mujoco>
    <option gravity="0 0 0">
        <flag contact="disable"/>
    </option>
    <compiler fusestatic="false">
    </compiler>
</mujoco>
"""


def export_urdf(robot, dst=None):
    tree = ET.fromstring(robot.to_xml_string())
    mujoco_config_node = ET.fromstring(mujoco_config_xml)
    tree.append(mujoco_config_node)
    if dst is not None:
        ET.ElementTree(tree).write(dst)
    return tree


def split_mjcf_to_body_mesh(mjcf_path, dst_dir):
    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    worldbody = root.find("worldbody")
    urdf = worldbody.find("body")
    urdf.attrib["pos"] = "-150 10 30"
    urdf.append(ET.Element("freejoint"))
    bodies = ET.Element("bodies")
    bodies.append(urdf)
    ET.ElementTree(bodies).write(os.path.join(dst_dir, "bodies.mjcf"))

    asset = root.find("asset")

    meshes = ET.Element("meshes")
    for mesh in asset.findall("mesh"):
        mesh_file = mesh.attrib["file"]
        mesh.attrib["file"] = os.path.join(dst_dir, "meshes", mesh_file)
        meshes.append(mesh)
    ET.ElementTree(meshes).write(os.path.join(dst_dir, "meshes.mjcf"))


if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument("--work_dir", default=r"F:\intel\interiorsim\pipeline\fetch")
    parser.add_argument("--urdf", default="fetch.urdf")
    args = parser.parse_args()

    raw_urdf = os.path.join(args.work_dir, "meshes", args.urdf)
    urdf_scaled = os.path.join(args.work_dir, "meshes", "temp_fetch_scaled.urdf")
    raw_mjcf = os.path.join(args.work_dir, "temp_fetch.mjcf")
    mjcf_scaled = os.path.join(args.work_dir, "temp_fetch_scaled.mjcf")
    mjcf_scaled2 = os.path.join(args.work_dir, "temp_fetch_scaled2.mjcf")
    robot = urdf.URDF.from_xml_file(raw_urdf)

    scale = 100
    for link in robot.links:
        for node in [link.collision, link.visual]:
            node.origin.xyz = (np.array(node.origin.xyz) * scale).tolist()
            if isinstance(node.geometry, urdf.Sphere):
                node.geometry.radius *= scale
            elif isinstance(node.geometry, urdf.Mesh):
                pass
    for joint in robot.joints:
        position = joint.origin.position
        print(joint.name, joint.origin.position)
        joint.origin.position = (np.array(position) * scale).tolist()
        if joint.dynamics is not None:
            # joint.dynamics.damping *= 0
            pass

    scaled_mjcf = export_urdf(robot, urdf_scaled)

    # convert to mjcf
    save_urdf_file_to_mjcf(raw_urdf, raw_mjcf)
    save_urdf_file_to_mjcf(urdf_scaled, mjcf_scaled)
    #
    scale_mjcf_mesh(mjcf_scaled, mjcf_scaled2, scale=scale)
    #
    # # export to bodies and meshes(with scale)
    split_mjcf_to_body_mesh(mjcf_scaled2, args.work_dir)

    print("Done.")
