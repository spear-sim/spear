import unreal


def validate_component(component):
    component_name = component.get_name()
    parent_components = component.get_parent_components()
    is_root_component = len(parent_components) == 0

    if isinstance(component, unreal.StaticMeshComponent):
        if not is_root_component:
            if component_name.startswith("mesh_") and component_name[len("mesh_"):].isdigit():
                pass
            elif component_name.startswith("group_") and component_name[len("group_"):].isdigit():
                # dummy component for dummy StaticMeshComponent
                pass
            else:
                unreal.log_warning(f'bad component naming : {component_name}')

        static_mesh = component.get_editor_property("static_mesh")
        if static_mesh is None:
            unreal.log_warning(f"empty static mesh : {component.get_name()}")
            body_instance = component.get_editor_property("body_instance")
            collision_profile_name = body_instance.get_editor_property("collision_profile_name")
            if collision_profile_name != "BlockAllDynamic":
                unreal.log_warning(f"invalid collision_profile_name: {collision_profile_name} for {component.get_name()}")
        else:
            # check collision preset
            use_default_collision = component.get_editor_property("use_default_collision")
            if not use_default_collision:
                unreal.log_warning(f"invalid use_default_collision=False: {component.get_name()}")

        # check material
    elif isinstance(component, unreal.LightComponent):
        if not is_root_component:
            if not component_name.startswith("light_"):
                unreal.log_warning(f'bad component naming : {component_name}')
            elif not component_name[len("light_"):].isdigit():
                unreal.log_warning(f'bad component naming : {component_name}')
    elif isinstance(component, unreal.PhysicsConstraintComponent):
        if not is_root_component:
            if not component_name.startswith("physical_constraint_"):
                unreal.log_warning(f'bad component naming : {component_name}')
            elif not component_name[len("physical_constraint_"):].isdigit():
                unreal.log_warning(f'bad component naming : {component_name}')

        # check if constraint component are properly setup
        component_name1 = component.get_editor_property("component_name1").get_editor_property("component_name")
        component_name2 = component.get_editor_property("component_name2").get_editor_property("component_name")

        if (component_name1 is None or component_name1 == "None") and (component_name2 is None or component_name2 == "None"):
            unreal.log_warning(f"invalid component for pcc {component_name}")
        else:
            actor1 = component.get_editor_property("constraint_actor1")
            if actor1 is None:
                actor1 = component.get_owner()
            if component_name1 is not None and component_name1 != "None":
                other_components = actor1.get_components_by_class(unreal.StaticMeshComponent)
                valid = False
                for other_component in other_components:
                    if other_component.get_name() == component_name1:
                        valid = True
                        break
                if not valid:
                    unreal.log_warning(f"invalid pcc component1 {component_name}: {component_name1}")
            actor2 = component.get_editor_property("constraint_actor2")
            if actor2 is None:
                actor2 = component.get_owner()
            if component_name2 is not None and component_name2 != "None":
                other_components = actor2.get_components_by_class(unreal.StaticMeshComponent)
                valid = False
                for other_component in other_components:
                    if other_component.get_name() == component_name2:
                        valid = True
                        break
                if not valid:
                    unreal.log_warning(f"invalid pcc component2 {component_name}: {component_name2}")

    # validate child components
    for i in range(0, component.get_num_children_components()):
        child_component = component.get_child_component(i)
        validate_component(child_component)


def validate_actor(actor):
    if isinstance(actor, unreal.Actor):
        # validate actor basic info
        # - check naming
        actor_label = str(actor.get_actor_label())
        actor_label_name = actor_label[:actor_label.rfind("_")]
        actor_label_index = actor_label[actor_label.find("_") + 1:]
        unreal.log(f"actor_label_name = {actor_label_name}, actor_label_index = {actor_label_index}")
        # - check folder
        actor_folder_path = str(actor.get_folder_path())
        actor_folder = actor_folder_path[actor_folder_path.rfind("/") + 1:]
        actor_folder_name = actor_folder[actor_folder.find("_") + 1:]
        actor_folder_index = actor_folder[:actor_folder.rfind("_")]

        if actor_label_name != actor_folder_name:
            unreal.log_warning(f"label mismatch actor_label_name = {actor_label_name} actor_folder_name = {actor_folder_name}")

        # - actor tag
        spear_semantic_tag = None
        for tag in actor.tags:
            tag_name = str(tag)
            if tag_name.find("spear:") == 0:
                if tag_name.find("spear:semantic:") == 0:
                    if spear_semantic_tag == None:
                        spear_semantic_tag = tag_name[len("spear:semantic:"):]
                    else:
                        unreal.log_warning(f"duplicated semantic tag: {spear_semantic_tag} - {tag_name}")
        if spear_semantic_tag is None:
            unreal.log_warning(f"actor not semantic labeled {actor.get_actor_label()}")
        elif spear_semantic_tag != actor_label_name:
            unreal.log_warning(f"label mismatch: spear_semantic_tag = {spear_semantic_tag} actor_label_name = {actor_label_name}")

        # recursively validate components
        validate_component(actor.root_component)


if __name__ == '__main__':
    EditorActorSubsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actors = EditorActorSubsystem.get_selected_level_actors()  # debug purpose
    if len(actors) == 0:
        actors = EditorActorSubsystem.get_all_level_actors()

    for actor in actors:
        actor_folder_path = str(actor.get_folder_path())
        # for now we only need to take care of Meshes folder, others actors are user custom data and ignore for validation
        if actor_folder_path.find("Meshes") == 0:
            validate_actor(actor)
        # Navigation, Rendering, Settings may have custom setting?
