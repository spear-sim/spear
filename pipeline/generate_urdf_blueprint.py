import argparse

import numpy as np
from urdf_parser_py import urdf

import unreal

so_subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)

unit_m_to_mm = 100


class UnrealUrdfBuilder():
    def __init__(self, filename):
        self.robot = urdf.URDF.from_xml_file(filename)

    def create_link(self, link_name, parent_handle):
        # create component
        link = self.robot.link_map[link_name]
        component, component_handle = self.create_component(parent_handle, unreal.StaticMeshComponent, link_name)

        component.body_instance.set_editor_property("simulate_physics", True)

        # set transform for non root component
        if link_name in self.robot.parent_map:
            parent_joint_name, parent_link_name = self.robot.parent_map[link_name]
            parent_joint = self.robot.joint_map[parent_joint_name]
            position = np.array(parent_joint.origin.xyz) * unit_m_to_mm
            rpy = np.array(parent_joint.origin.rpy)
            rotation = unreal.Rotator(rpy[2], rpy[0], rpy[1])

            component.set_editor_property("relative_location", position.tolist())
            component.set_editor_property("relative_rotation", rotation)

        # set collision mesh TODO import non-unreal files
        geometry = link.collision.geometry
        if isinstance(geometry, urdf.Mesh):
            static_mesh = unreal.EditorAssetLibrary.load_asset(geometry.filename)
            component.set_editor_property("static_mesh", static_mesh)
        elif isinstance(geometry, urdf.Sphere):
            static_mesh = unreal.EditorAssetLibrary.load_asset("/Script/Engine.StaticMesh'/Engine/BasicShapes/Sphere.Sphere'")
            component.set_editor_property("static_mesh", static_mesh)
            component.set_editor_property("relative_scale3d", (np.array([1, 1, 1]) * 0.02 * unit_m_to_mm * geometry.radius).tolist())
        else:
            print("TOOD unknown geometry", type(geometry))
        # TODO temp
        if "caster" in link_name:
            simple_material = unreal.EditorAssetLibrary.load_asset("/UrdfRobot/Fetch_V1/Materials/M_slippery.M_slippery")
            component.set_material(0, simple_material)

        # setup children components if any
        if link_name in self.robot.child_map:
            for (child_joint_name, child_link_name) in self.robot.child_map[link_name]:
                joint, joint_handle = self.create_joint(child_joint_name, component_handle)
                self.create_link(child_link_name, component_handle)
        return component, component_handle

    def create_joint(self, joint_name, parent_handle):
        joint = self.robot.joint_map[joint_name]
        component, component_handle = self.create_component(parent_handle, unreal.PhysicsConstraintComponent, joint_name)
        position = np.array(joint.origin.xyz) * unit_m_to_mm
        rpy = np.array(joint.origin.rpy)
        rotation = unreal.Rotator(rpy[2], rpy[0], rpy[1])
        joint_axis = unreal.Vector(joint.axis[0], joint.axis[1], joint.axis[2])
        rotation = unreal.MathLibrary.make_rot_from_x(joint_axis.rotate(rotation))

        component.set_editor_property("relative_location", position.tolist())
        component.set_editor_property("relative_rotation", rotation)

        ratio = 100 * 180 / np.pi * 1

        component.set_disable_collision(True)
        component.get_editor_property("constraint_instance").get_editor_property("profile_instance").set_editor_property("parent_dominates", True)

        if joint.type == "fixed":
            component.set_angular_swing1_limit(unreal.AngularConstraintMotion.ACM_LOCKED, 0)
            component.set_angular_swing2_limit(unreal.AngularConstraintMotion.ACM_LOCKED, 0)
            component.set_angular_twist_limit(unreal.AngularConstraintMotion.ACM_LOCKED, 0)
        elif joint.type == "revolute" or joint.type == "continuous":
            component.set_angular_swing1_limit(unreal.AngularConstraintMotion.ACM_LOCKED, 0)
            component.set_angular_swing2_limit(unreal.AngularConstraintMotion.ACM_LOCKED, 0)
            # component.set_angular_twist_limit(unreal.AngularConstraintMotion.ACM_LOCKED, 0)

            if "wheel_joint" in joint_name:
                component.set_angular_orientation_drive(False, True)
                component.set_angular_drive_mode(unreal.AngularDriveMode.TWIST_AND_SWING)
                component.set_angular_velocity_drive_twist_and_swing(True, False)
                component.set_angular_drive_params(0, 100 * ratio, 100 * 10 * ratio)

                component.get_editor_property("constraint_instance").get_editor_property("profile_instance").set_editor_property("parent_dominates", False)
            else:
                component.set_angular_orientation_drive(False, True)
                component.set_angular_drive_mode(unreal.AngularDriveMode.TWIST_AND_SWING)
                component.set_angular_velocity_drive_twist_and_swing(True, False)
                component.set_angular_drive_params(100 * ratio, 0.4 * ratio, 100 * 10 * ratio)

        elif joint.type == "prismatic":
            component.set_angular_swing1_limit(unreal.AngularConstraintMotion.ACM_LOCKED, 0)
            component.set_angular_swing2_limit(unreal.AngularConstraintMotion.ACM_LOCKED, 0)
            component.set_angular_twist_limit(unreal.AngularConstraintMotion.ACM_LOCKED, 0)
            component.set_linear_x_limit(unreal.LinearConstraintMotion.LCM_FREE, 0)

        component_name1 = unreal.ConstrainComponentPropName()
        component_name1.set_editor_property("component_name", joint.child)
        component.set_editor_property("component_name1", component_name1)
        component_name2 = unreal.ConstrainComponentPropName()
        component_name2.set_editor_property("component_name", joint.parent)
        component.set_editor_property("component_name2", component_name2)

        return component, component_handle

    def create_component(self, parent_handle, subobject_class, name=None):
        sub_handle, fail_reason = so_subsystem.add_new_subobject(
            params=unreal.AddNewSubobjectParams(
                parent_handle=parent_handle,
                new_class=subobject_class,
                blueprint_context=self.actor))
        if name is not None:
            so_subsystem.rename_subobject(handle=sub_handle, new_name=unreal.Text(name))

        BFL = unreal.SubobjectDataBlueprintFunctionLibrary
        new_component = BFL.get_object(BFL.get_data(sub_handle))
        return new_component, sub_handle

    def create_blueprint(self, asset_name, package_path):
        # create or load blueprint asset
        self.actor = unreal.EditorAssetLibrary.load_asset(f"{package_path}/{asset_name}.{asset_name}")
        if self.actor is None:
            # see https://forums.unrealengine.com/t/creating-blueprint-assets-hierarchies-with-python/115929/24
            factory = unreal.BlueprintFactory()
            factory.set_editor_property("ParentClass", unreal.Actor)

            asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
            self.actor = asset_tools.create_asset(asset_name, package_path, None, factory)

        # create urdf component hierarchy
        root_data_handle = so_subsystem.k2_gather_subobject_data_for_blueprint(self.actor)[0]
        root = self.robot.get_root()
        base_link_component, base_link_component_handle = self.create_link(root, root_data_handle)

        # TODO replace base_link as root component
        camera_component, _ = self.create_component(base_link_component_handle, unreal.load_class(None, "/Script/CoreUObject.Class'/Script/Engine.CameraComponent'"))
        camera_component.set_editor_property("relative_location", [10, 0, 120])

        self.create_component(root_data_handle, unreal.load_class(None, "/Script/CoreUObject.Class'/Script/SpComponents.CameraSensorComponent'"))
        self.create_component(root_data_handle, unreal.load_class(None, "/Script/CoreUObject.Class'/Script/SpCore.StableNameComponent'"))

        # save asset
        unreal.EditorAssetLibrary.save_loaded_asset(self.actor)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=r"F:\intel\interiorsim\python\spear\urdf\fetch_no_spear.urdf")
    parser.add_argument("--blueprint_name", default="BP_Fetch")
    parser.add_argument("--package_path", default="/Game/Agents")
    args = parser.parse_args()

    builder = UnrealUrdfBuilder(args.urdf)
    builder.create_blueprint(args.blueprint_name, args.package_path)

    print("Done.")
