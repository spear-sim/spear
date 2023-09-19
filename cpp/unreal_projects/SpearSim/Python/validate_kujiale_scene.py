import unreal


def validate_material(material):
    warning_count = 0
    base_material = material.get_base_material()

    if base_material.get_name() == "M_BaseMaterial" or base_material.get_name() == "M_TranslucentMaterial":
        material_name = str(material.get_name())
        valid_name = True
        if not material_name.startswith("MI_"):
            valid_name = False
        if not valid_name:
            unreal.log_warning(f"invalid material naming : {material_name}")
            warning_count += 1

        material_path = str(material.get_path_name())
        valid_folder = True
        folder_structure = material_path.split("/")
        if len(folder_structure) != 7:
            valid_folder = False
        elif folder_structure[1] != "Game":
            valid_folder = False
        elif folder_structure[2] != "Scenes":
            valid_folder = False
        elif folder_structure[3].startswith("kujiale_") and not folder_structure[3][len("kujiale_"):].isdigit():
            valid_folder = False
        elif folder_structure[4] != "Materials":
            valid_folder = False

        if not valid_folder:
            unreal.log_warning(f"invalid material path : {material_path}")
            warning_count += 1
    return warning_count


def valid_static_mesh_kujiale(static_mesh):
    static_mesh_name = str(static_mesh.get_name())
    valid_name = True
    if not static_mesh_name.startswith("SM_"):
        valid_name = False
    elif static_mesh_name != "SM_Dummy":
        index_split_pos = static_mesh_name.rfind("_")
        if index_split_pos > len(static_mesh_name):
            valid_name = False
        else:
            index = static_mesh_name[index_split_pos + 1:]
            if len(index) != 4 or not index.isdigit():
                valid_name = False
    if not valid_name:
        unreal.log_warning(f"invalid static mesh naming : {static_mesh_name}")

    static_mesh_path = static_mesh.get_path_name()
    valid_folder = True
    if static_mesh_name != "SM_Dummy":
        folder_structure = static_mesh_path.split("/")
        if len(folder_structure) != 8:
            valid_folder = False
        elif folder_structure[1] != "Game":
            valid_folder = False
        elif folder_structure[2] != "Scenes":
            valid_folder = False
        elif folder_structure[3].startswith("kujiale_") and not folder_structure[3][len("kujiale_"):].isdigit():
            print("123", folder_structure[3][len("kujiale_"):])
            valid_folder = False
        elif folder_structure[4] != "Meshes":
            valid_folder = False
        elif folder_structure[5] != "Base" and folder_structure[5] != "Clutter":
            valid_folder = False
    if not valid_folder:
        unreal.log_warning(f"invalid static mesh path : {static_mesh_path}")
    # print('static_mesh_name', static_mesh_name, static_mesh_path)


def validate_component_kujiale(component):
    warning_count = 0
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
                warning_count += 1

        static_mesh = component.get_editor_property("static_mesh")
        valid_static_mesh_kujiale(static_mesh)
        for material in component.get_editor_property("override_materials"):
            if static_mesh.get_name() != "SM_Dummy":
                if material is None:
                    unreal.log_warning(f"empty material?  {component.get_name()}")
                    warning_count += 1
                else:
                    warning_count += validate_material(material)

    # validate child components
    for i in range(0, component.get_num_children_components()):
        child_component = component.get_child_component(i)
        warning_count += validate_component_kujiale(child_component)
    return warning_count


def validate_actor_kujiale(actor):
    if isinstance(actor, unreal.Actor):
        # recursively validate components
        warning_count = validate_component_kujiale(actor.root_component)

        if warning_count > 0:
            unreal.log_warning(f"warning for {actor.get_actor_label()} : {warning_count}")


# validate actor basic info
if __name__ == '__main__':
    EditorActorSubsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actors = EditorActorSubsystem.get_selected_level_actors()  # debug purpose
    if len(actors) == 0:
        actors = EditorActorSubsystem.get_all_level_actors()

    for actor in actors:
        actor_folder_path = str(actor.get_folder_path())
        # for now we only need to take care of Meshes folder, others actors are user custom data and ignore for validation
        if actor_folder_path.find("Meshes") == 0:
            validate_actor_kujiale(actor)
        # Navigation, Rendering, Settings may have custom setting?
