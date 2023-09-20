# MuJoCo_Export_Pipeline

## Assumptions
- The floor geometry must have the floor spread across the X-Y plane, and Z axis pointing upwards.
- XY plane must be at the level of the top surface of the floor.
- Within each top-level Actor, no two StaticMeshComponents can have the same name, and no two PhysicsConstraintComponents can have the same name.

## TODO
- Multiple joints with the same parent and child are not permitted
- Parse "simulate phyiscs" flags in UE to determine freejoints, and if it is set for any component other than the DefaultSceneRoot and children of PhysicsContstraintComponents.
- Map Actor names <> actor labels
- Assert actor labels and labels of component names within actors are unique

## Miscellaneous
```python
import unreal


ss = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world = ss.get_editor_world()
all_actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
door = [a for a in all_actors if a.get_actor_label() == 'door_00'][0]
door_cs = [c for c in door.get_components_by_class(unreal.PhysicsConstraintComponent)]
c = door_cs[0]
profile = c.get_editor_property('constraint_instance').get_editor_property('profile_instance')
```