import unreal
import os
from collections import namedtuple
import json
import math
import time

osp = os.path
ss = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world = ss.get_editor_world()
export_dir = osp.expanduser(osp.join('~', 'research', 'MuJoCo_Export_Pipeline', 'scenes', 'kujiale_0000', 'ue_export')) 


def export_meshes(actor):
  unreal.EditorLevelLibrary.set_selected_level_actors([actor, ])
  name = actor.get_actor_label()
  filename = osp.join(export_dir, f'{name}.gltf')
  task = unreal.AssetExportTask()
  task.set_editor_property('selected', True)
  task.set_editor_property('automated', True)
  task.set_editor_property('prompt', False)
  task.set_editor_property('object', world)
  task.set_editor_property('filename', filename)
  print(f'Writing actor {name} to {filename}...')
  count = 0
  N = 10
  while not unreal.Exporter.run_asset_export_task(task):
    print(f'Retry {count+1} / {N}')
    time.sleep(0.5)
    if count == 10:
      break
  else:
    return True
  return False


PhysicsConstraint = namedtuple('PhysicsConstraint', ['parent', 'child', 'name', 'range', 'axis', 'type'])


if __name__ == '__main__':
  # Iterate through the actors
  body_properties = {}
  for actor in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor):
    if 'HDRIBackdrop' in actor.get_actor_label():
      continue
    
    # check if actor has geometry
    for c in actor.get_components_by_class(unreal.StaticMeshComponent):
      if c.get_editor_property('static_mesh').get_name() != 'SM_Dummy':
        export_meshes(actor)
        break
    else:
      print(f'Actor {actor.get_actor_label()}, does not have geometry')
      continue
    
    if isinstance(actor, unreal.StaticMeshActor):
      moving = actor.get_editor_property('static_mesh_component').get_editor_property('body_instance').get_editor_property('simulate_physics')
    else: # TODO(samarth) get the actual flag for the actuated body
      moving = False
    body_properties[actor.get_actor_label()] = {
      'root_component_name': actor.get_editor_property('root_component').get_name(),
      'moving': moving,
    }
    
    constraint_dicts = []
    for component in actor.get_components_by_class(unreal.ActorComponent):
      if not isinstance(component, unreal.PhysicsConstraintComponent):
        continue
      
      r = component.get_editor_property('relative_rotation')
      rot = unreal.Rotator(r.roll, r.pitch, r.yaw)
      
      parent = str(component.get_editor_property('component_name1').get_editor_property('component_name'))
      if (parent == 'DefaultSceneRoot') or (parent == 'StaticMeshComponent0'):
        parent = actor.get_actor_label()
      child = str(component.get_editor_property('component_name2').get_editor_property('component_name'))
      
      constraint_instance = component.get_editor_property('constraint_instance')
      angular_offset = constraint_instance.get_editor_property('angular_rotation_offset')
      constraint_profile = constraint_instance.get_editor_property('profile_instance')
      
      limit = 0
      offset = 0
      
      linear_limit = constraint_profile.get_editor_property('linear_limit')
      limit = linear_limit.get_editor_property('limit') / 100.0
      
      n_prismatic_dofs = 0
      pc_args = [parent, child, ]
      if linear_limit.get_editor_property('x_motion') != unreal.LinearConstraintMotion.LCM_LOCKED:
        n_prismatic_dofs += 1
        axis = unreal.Vector.FORWARD
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_x_prismatic',
          [offset-limit, offset+limit],
          [axis.x, -axis.y, axis.z],
          'slide'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
      pc_args = [parent, child, ]
      if linear_limit.get_editor_property('y_motion') != unreal.LinearConstraintMotion.LCM_LOCKED:
        n_prismatic_dofs += 1
        axis = unreal.Vector.RIGHT
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_y_prismatic',
          [offset-limit, offset+limit],
          [axis.x, -axis.y, axis.z],
          'slide'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
      pc_args = [parent, child, ]
      if linear_limit.get_editor_property('z_motion') != unreal.LinearConstraintMotion.LCM_LOCKED:
        n_prismatic_dofs += 1
        axis = unreal.Vector.UP
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_z_prismatic',
          [offset-limit, offset+limit],
          [axis.x, -axis.y, axis.z],
          'slide'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
      if n_prismatic_dofs > 1:
        raise Warning(f'{actor.get_actor_label()} has a multi-axis prismatic constraint')
      
      n_revolute_dofs = 0
      cone_limit = constraint_profile.get_editor_property('cone_limit')
      pc_args = [parent, child, ]
      if (cone_limit.get_editor_property('swing1_motion') != unreal.AngularConstraintMotion.ACM_LOCKED):
        n_revolute_dofs += 1
        if n_prismatic_dofs > 1:
          raise Warning(f'{actor.get_actor_label()} already has {n_prismatic_dofs} prismatic joints')
        limit = math.pi * cone_limit.get_editor_property('swing1_limit_degrees') / 180.0
        offset = math.pi * angular_offset.yaw / 180.0
        axis = unreal.Vector.UP
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_z_revolute',
          [offset-limit, offset+limit],
          [axis.x, -axis.y, axis.z],
          'hinge'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
      pc_args = [parent, child, ]
      if (cone_limit.get_editor_property('swing2_motion') != unreal.AngularConstraintMotion.ACM_LOCKED):
        n_revolute_dofs += 1
        if n_prismatic_dofs > 1:
          raise ValueError(f'{actor.get_actor_label()} already has {n_prismatic_dofs} prismatic joints')
        limit = math.pi * cone_limit.get_editor_property('swing2_limit_degrees') / 180.0
        offset = math.pi * angular_offset.pitch / 180.0
        axis = unreal.Vector.RIGHT
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_y_revolute',
          [offset-limit, offset+limit],
          [axis.x, -axis.y, axis.z],
          'hinge'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
      twist_limit = constraint_profile.get_editor_property('twist_limit')
      pc_args = [parent, child, ]
      if twist_limit.get_editor_property('twist_motion') != unreal.AngularConstraintMotion.ACM_LOCKED:
        n_revolute_dofs += 1
        if n_prismatic_dofs > 1:
          raise ValueError(f'{actor.get_actor_label()} already has {n_prismatic_dofs} prismatic joints')
        limit = math.pi * twist_limit.get_editor_property('twist_limit_degrees') / 180.0
        offset = math.pi * angular_offset.roll / 180.0
        axis = unreal.Vector.FORWARD
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_x_revolute',
          [offset-limit, offset+limit],
          [axis.x, -axis.y, axis.z],
          'hinge'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
    
    if len(constraint_dicts) > 0:
      filename = osp.join(export_dir, f'{actor.get_actor_label()}_joints.json')
      with open(filename, 'w') as f:
        json.dump(constraint_dicts, f, indent=4)
      print(f'{filename} written')
    
  filename = osp.join(export_dir, 'actors_information.json')
  with open(filename, 'w') as f:
      json.dump(body_properties, f, indent=4)
  print(f'{filename} written')