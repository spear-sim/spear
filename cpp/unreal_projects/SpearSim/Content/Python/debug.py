import unreal

# Get the currently selected level
level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).get_current_level()

if level:
    # Get all actors in the level
    actors = unreal.GameplayStatics.get_all_actors_of_class(level, unreal.Actor)

    for actor in actors:
        print("Actor Name:", actor.get_actor_label())
        static_mesh_components = actor.get_components_by_class(unreal.StaticMeshComponent)
        scene_components = actor.get_components_by_class(unreal.SceneComponent)
        if static_mesh_components and scene_components:
            # special case, ignore HDRIBackdrop actor
            if actor.get_actor_label() == "HDRIBackdrop":
                continue
            print("     Valid Component: Has either StaticMeshComponent or SceneComponent")
            so_subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
            root_sub_object = so_subsystem.k2_gather_subobject_data_for_instance(actor)[0]
            new_sub_object = so_subsystem.add_new_subobject(unreal.AddNewSubobjectParams(
                parent_handle=root_sub_object,
                new_class=unreal.SpearComponent,
            ))
            # print(f"created {new_sub_object}")
        else:
            print("     Invalid component: No StaticMeshComponents or SceneComponents")

    for actor in actors:
        print("Actor Name:", actor.get_actor_label())
        spear_components = actor.get_components_by_class(unreal.SpearComponent)
        for spear_component in spear_components:
            # print(unreal.SpearComponent.__dict__)
            spear_component.set_editor_property("parent_actor_label_name", actor.get_actor_label())
else:
    print("No level selected in the editor.")