import numpy as np
import pathlib
import posixpath
import unreal
import spear


def validate_level(level):

    #
    # TODO:
    # We name each scene using a lower_case_with_underscore naming convention and a four
    # digit suffix (e.g., "apartment_0000", "apartment_0001", etc).
    #

    pass


def validate_content_browser(level):

    #
    # TODO:
    # There should be exactly one umap file with the same name as the scene in the scene's
    # Maps directory. Our code imposes this requirement to support loading scenes by name.
    #

    #
    # TODO:
    # The "Content Browser" should not contain unreferenced assets, except for in the
    # "Debug" directory.
    #

    pass


def validate_outliner(level):

    editor_actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

    #
    # TODO: use args to decide whether or not to use selected actors
    #
    # if args.validate_selected_actors:
    #     actors = editor_actor_subsystem.get_selected_level_actors()
    # else:
    #     actors = editor_actor_subsystem.get_all_level_actors()
    #

    # actors = sorted(editor_actor_subsystem.get_selected_level_actors(), key=(lambda actor: get_debug_string_actor(actor)))
    actors = sorted(editor_actor_subsystem.get_all_level_actors(), key=(lambda actor: get_debug_string_actor(actor)))

    for a in actors:
        print(get_debug_string_actor(a))

    wall = [ a for a in actors if get_debug_string_actor(a) == "Meshes/03_cabinet/Cabinet" ][0]
    print(wall)

    # attributes = dir(wall)
    # properties = {}
    # for k in attributes:
    #     try:
    #         v = wall.get_editor_property(k)
    #         properties[k] = v
    #     except:
    #         pass
    # print(properties)

    # for k in sorted(properties.keys()):
    #     print(k, properties[k])

    components = wall.get_components_by_class(unreal.ActorComponent)
    for component in components:
        if type(component) == unreal.PhysicsConstraintComponent:
            print(get_debug_string_component(component))

    components = components[3].get_children_components(include_all_descendants=True)
    print(components)
    for component in components:
        print(get_debug_string_component(component))

    assert False

    smc = wall.get_editor_property("static_mesh_component")
    print(smc)

    attributes = dir(smc)
    properties = {}
    for k in attributes:
        try:
            v = smc.get_editor_property(k)
            properties[k] = v
        except:
            pass
    print(properties)

    for k in sorted(properties.keys()):
        print(k, properties[k])


def validate_actor(level, actor):

    #
    # Actors that are useful for debugging are kept here.
    #
    # All Actors that represent non-trivial scene geometry are kept here.
    #
    # All navigation-related Actors are kept here. There must be at least one RecastNavMesh 
    # Actor in the scene in order to use the navmesh functionality in our Python API.
    #
    # All rendering-related Actors are kept here. For convenience, we allow content here to 
    # deviate from our usual naming conventions.
    #
    # All settings-related Actors are kept here.  For convenience, we allow content here to 
    # deviate from our usual naming conventions.
    #
    # All of our custom Widgets are kept here.
    #

    path = pathlib.PurePosixPath(str(actor.get_folder_path())).parts

    if len(path) < 1:
        spear.log("WARNING: Unexpected path: ", get_debug_string_actor(actor))
        return

    if path[0] not in ["Debug", "Meshes", "Navigation", "Rendering", "Settings", "Widgets"]:
        spear.log("WARNING: Unexpected path: ", get_debug_string_actor(actor))
        return

    if path[0] in ["Meshes", "Rendering"]:
        if len(path) != 2:
            spear.log("WARNING: Unexpected path: ", get_debug_string_actor(actor))
            return

    if path[0] in ["Navigation", "Settings", "Widgets"]:
        if len(path) != 1:
            spear.log("WARNING: Unexpected path: ", get_debug_string_actor(actor))
            return

    if path[0] == "Meshes":

        #
        # We create a subdirectory for each semantic category using a lower_case_with_underscore
        # naming convention and a four digit prefix that represents the category's integer ID.
        # We assign each Actor to a particular semantic category by placing the Actor in that
        # category's subdirectory. Semantic ID 0 is reserved and should not be used here. If an
        # Actor has multiple StaticMeshComponents that should have different categories, the
        # category of individual StaticMeshComponents can be overridden. In this case, the Actor
        # should be assigned to whatever subdirectory is most convenient, i.e., requires the least
        # number of per-component overrides.
        #
        # We choose to encode semantic annotations via this directory structure, because it
        # makes it especially easy to browse each scene in the Unreal Editor. For example,
        # using this directory structure, a user can easily select all objects in a particular
        # semantic category and make them invisible, thereby making it easier to browse the
        # rest of the scene.
        #

        semantic_category = path[1].split("_")

        if len(semantic_category) < 2:
            spear.log("WARNING: Unexpected path: ", get_debug_string_actor(actor))
            return

        if len(semantic_category[0]) != 4 or not semantic_category[0].isdigit():
            spear.log("WARNING: Unexpected path: ", get_debug_string_actor(actor))
            return

        if int(semantic_category[0]) == 0:
            spear.log("WARNING: Unexpected path: ", get_debug_string_actor(actor))
            return

        actor_label = actor.get_actor_label().split("_")
        if actor_label[:-1] != semantic_category[1:]:
            spear.log("WARNING: Unexpected path: ", get_debug_string_actor(actor))
            return

        #
        # The type of each actor in my_scene_0000/Meshes should be StaticMeshActor if it is not
        # articulated, and Actor if it is articulated. It is necessary to use the Actor type for
        # articulated actors so they can be correctly physically simulated. Although it is not
        # strictly necessary to use the StaticMeshActor type for non-articulated actors,
        # StaticMeshActors and Actors have different icons in the Outliner pane, so we use
        # different types to visually differentiate them.
        #

        if type(actor) != unreal.Actor and type(actor) != unreal.StaticMeshActor:
            spear.log("WARNING: Unexpected type: ", get_debug_string_actor(actor), " (", get_debug_string_type(type(actor)), ")")
            return

        #
        # The pivot location of each StaticMeshActor should be set according to the
        # following rules. The xy-coordinates of the actor's pivot should equal the
        # xy-coordinates of the actor's axis-aligned bounding box center, and the
        # z-coordinate of the pivot should equal the minimum z-coordinate of its
        # axis-aligned bounding box. Having a consistent convention here makes it easier
        # to programmatically spawn objects, and this is the convention for various props
        # that ship with the Unreal Engine (e.g., the props in our "debug_0000" scene).
        #
        # All the rules for StaticMeshActors described above also apply to Actors.
        #

        # ignore components from bounds computation if they don't have a mesh assigned, or if their assigned mesh is SM_Dummy
        ignore_components = [
            c for c in actor.get_components_by_class(unreal.StaticMeshComponent) if
            c.get_editor_property("static_mesh") is None or
            c.get_editor_property("static_mesh").get_path_name() == "/Game/Common/Meshes/SM_Dummy.SM_Dummy" ]

        # for each ignored component, cache "use_default_collision" and "collision_enabled" properties
        ignore_use_default_collision = {
            c.get_name(): c.get_editor_property("use_default_collision") for c in ignore_components }
        ignore_collision_enabled = {
            c.get_name(): c.get_editor_property("body_instance").get_editor_property("collision_enabled") for c in ignore_components }

        # disable collision on each ignored component
        for c in ignore_components:
            c.set_editor_property("use_default_collision", False)
            c.get_editor_property("body_instance").set_editor_property("collision_enabled", unreal.CollisionEnabled.NO_COLLISION)

        # compute bounds
        bounds_origin, bounds_half_extent = actor.get_actor_bounds(only_colliding_components=True, include_from_child_actors=False)

        # for each ignored component, restore "use_default_collision" and "collision_enabled" properties
        for c in ignore_components:
            c.set_editor_property("use_default_collision", ignore_use_default_collision[c.get_name()])
            c.get_editor_property("body_instance").set_editor_property("collision_enabled", ignore_collision_enabled[c.get_name()])

        # check pivot location against computed bounds
        eps = 0.000001
        bounds_origin = vector_to_numpy(bounds_origin)
        bounds_half_extent = vector_to_numpy(bounds_half_extent)
        pivot_desired = np.array([bounds_origin[0], bounds_origin[1], bounds_origin[2] - bounds_half_extent[2]])
        pivot_actual = vector_to_numpy(actor.get_actor_location())
        if np.linalg.norm(pivot_desired - pivot_actual) > eps:
            np.set_printoptions(suppress=True)
            spear.log(
                "WARNING: Unexpected pivot: ", get_debug_string_actor(actor),
                " (desired=", pivot_desired, ", actual=", pivot_actual, ")")
            return

        if type(actor) == unreal.Actor:
            validate_meshes_actor(level, actor)
        if type(actor) == unreal.StaticMeshActor:
            validate_meshes_static_mesh_actor(level, actor)

    if path[0] == "Rendering":
        if path[1] not in ["Fog", "Lights", "Sky"]:
            spear.log("WARNING: Unexpected path: ", get_debug_string_actor(actor))
            return


def validate_meshes_actor(level, actor):

    if len(actor.get_components_by_class(unreal.PhysicsConstraintComponent)) == 0:
        spear.log("WARNING: Expected physics constraint: ", get_debug_string_actor(actor))
        return

    components = actor.get_components_by_class(unreal.ActorComponent)
    for component in components:
        if type(component) == unreal.LightComponent:
            validate_light_component(level, actor, component)
        if type(component) == unreal.PhysicsConstraintComponent:
            validate_physics_constraint_component(level, actor, component)
        if type(component) == unreal.SceneComponent:
            validate_scene_component(level, actor, component)
        if type(component) == unreal.StaticMeshComponent:
            validate_static_mesh_component(level, actor, component)


def validate_meshes_static_mesh_actor(level, actor):

    if len(actor.get_components_by_class(unreal.PhysicsConstraintComponent)) != 0:
        spear.log("WARNING: Unexpected physics constraint: ", get_debug_string_actor(actor))
        return

    components = actor.get_components_by_class(unreal.ActorComponent)
    for component in components:
        if type(component) == unreal.LightComponent:
            validate_light_component(level, actor, component)
        if type(component) == unreal.StaticMeshComponent:
            validate_static_mesh_component(level, actor, component)


def validate_light_component(level, actor, component):

    if len(component.get_parent_components()) == 0:
        spear.log("WARNING: Unexpected component: ", get_debug_string_actor(actor), ".", component)
        return

    component_name = component.get_name().split("_")

    if len(component_name) != 2:
        spear.log("WARNING: Unexpected component name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
        return

    if component_name[0] != "light":
        spear.log("WARNING: Unexpected component name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
        return

    if len(component_name[1]) != 4 or not component_name[1].isdigit():
        spear.log("WARNING: Unexpected component name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
        return


def validate_physics_constraint_component(level, actor, component):

    if len(component.get_parent_components()) == 0:
        spear.log("WARNING: Unexpected component: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
        return

    component_name = component.get_name().split("_")

    if len(component_name) != 3:
        spear.log("WARNING: Unexpected component name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
        return

    if component_name[0:2] != ["physics", "constraint"]:
        spear.log("WARNING: Unexpected component name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
        return

    if len(component_name[2]) != 4 or not component_name[2].isdigit():
        spear.log("WARNING: Unexpected component name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
        return

    component_name1 = component.get_editor_property("component_name1")
    component_name2 = component.get_editor_property("component_name2")

    if component_name1 is None or component_name2 is None:
        spear.log("WARNING: Unexpected physics constraint name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
        return

    component_name1_name = component_name1.get_editor_property("component_name")
    component_name2_name = component_name2.get_editor_property("component_name")

    if component_name1_name == "None" or component_name2_name == "None":
        spear.log(
            "WARNING: Unexpected physics constraint name 1: ",
            get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", component_name1_name, ")")
        return

    #
    # A UrdfJointComponent represents a joint, and is implemented as a derived class of
    # PhysicsConstraintComponent with additional state and functionality. Each joint connects
    # a parent and a child component, and the joint component itself should be a sibling of
    # the child component it is connecting. In our guidelines, we do not allow joints to
    # connect sibling components, or grandparents and grandchildren, even though this would
    # be permitted in Unreal, because this type of joint cannot always be simulated
    # efficiently in other physics engines.
    #

    #
    # When configuring a joint, the convention in Unreal is for "Component 1" to be the child and
    # "Component 2" to be the parent. This convention is relevant, e.g., when the "Parent Dominates"
    # option is enabled on the joint.
    #

    parent = component.get_attach_parent()

    if component_name1_name not in [ c.get_name() for c in parent.get_children_components(include_all_descendants=False) ]:
        spear.log(
            "WARNING: Unexpected physics constraint name 1, ",
            get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", component_name1_name, ")")
        return

    if component_name2_name != parent.get_name():
        spear.log(
            "WARNING: Unexpected physics constraint name 2: ",
            get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", component_name2_name, ")")
        return


def validate_scene_component(level, actor, component):

    #
    # Each Actor has a root component named "DefaultSceneRoot".
    #

    if len(component.get_parent_components()) != 0:
        spear.log("WARNING: Unexpected component: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
        return

    if component.get_name() != "DefaultSceneRoot":
        spear.log("WARNING: Unexpected component name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
        return


def validate_static_mesh_component(level, actor, component):

    #
    # For each StaticMeshComponent, the "Collision Presets" option should be set to
    # "Default". This configures the collision behavior of the component to be
    # determined by the options on the underlying mesh, rather than the options on
    # the component itself.
    #

    if not component.get_editor_property("use_default_collision"):
        spear.log("WARNING: Unexpected collision presets: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
        return

    component_name = component.get_name().split("_")

    #
    # Each StaticMeshActor has a root component named "StaticMeshComponent0".
    #

    if len(component.get_parent_components()) == 0:
        if component_name[0] != "StaticMeshComponent0":
            spear.log("WARNING: Unexpected component name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
            return

    #
    # We expect Actors to be articulated, and therefore they will be composed of multiple
    # StaticMeshComponents. The additional StaticMeshComponents should be added as children
    # of a parent StaticMeshComponent. Each StaticMeshComponent that is only used to group
    # other StaticMeshComponents together should be should be named "group_0000",
    # "group_0001", etc. Each StaticMeshComponent that contains non-trivial scene geometry 
    # should be named "mesh_0000", "mesh_0001", etc.
    #

    #
    # Assets that are referenced in multiple scenes are kept in the Common directory.
    # If a collection of scenes needs to refer to additional common assets, those assets
    # should be kept in sibling directories to Common (e.g., the kujiale scenes refer to
    # assets in {Kujiale, Megascans, MSPresets}, the debug scenes refer to assets in 
    # {StarterContent}, etc).
    #
    # Each scene inside the Scenes directory is allowed to refer to assets in common
    # directories, but is not allowed to refer to assets in other scene directories,
    # e.g., my_scene_0000 is not allowed to refer to assets in the my_scene_0001 directory
    # below. This restriction makes it easier to support an editing workflow where a
    # developer only needs to download a single subdirectory in order to obtain a
    # complete self-contained copy of a scene.
    #

    if len(component.get_parent_components()) != 0:

        if len(component_name) != 2:
            spear.log("WARNING: Unexpected component name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
            return

        if component_name[0] not in ["group", "mesh"]:
            spear.log("WARNING: Unexpected component name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
            return

        if len(component_name[1]) != 4 or not component_name[1].isdigit():
            spear.log("WARNING: Unexpected component name: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
            return

    #
    # If the parent StaticMeshComponent exists only to group other child StaticMeshComponents
    # together, and it is desired for the group to be physically simulated, then the mesh assigned
    # to the parent component should be "/Game/Common/Meshes/SM_Dummy", and the "Simulate Phyiscs"
    # option should be enabled for the parent but not for the children. This is necessary for the
    # group to be correctly physically simulated.
    #

    if component_name[0] == "StaticMeshComponent0" or component_name[0] == "group":

        if component.is_simulating_physics():
            static_mesh = component.get_editor_property("static_mesh")
            if static_mesh is None:
                spear.log(
                    "WARNING: Unexpected static mesh: ",
                    get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_static_mesh(static_mesh), ")")
                return
            static_mesh_path = pathlib.PurePosixPath(static_mesh.get_path_name()).parts
            if static_mesh_path != ("/", "Game", "Common", "Meshes", "SM_Dummy.SM_Dummy"):
                spear.log(
                    "WARNING: Unexpected static mesh: ",
                    get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_static_mesh(static_mesh), ")")
                return

        materials = component.get_editor_property("override_materials")
        for material in materials:
            if material is not None:
                spear.log(
                    "WARNING: Unexpected material: ",
                    get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_material(material), ")")
                return

    #
    # If the StaticMeshActor is composed of multiple StaticMeshComponents, the additional
    # StaticMeshComponents can be added as children of the top-level StaticMeshComponent,
    # using the names "mesh_0000", "mesh_0001", etc. This approach enables the child
    # StaticMeshComponents to be easily selected and moved together in the editor, and
    # enables them to move together during a physics simulation, without needing to
    # explicitly merge all the actor's geometry together into a single StaticMeshComponent.
    #

    if component_name[0] == "mesh":

        #
        # We organize "base" meshes according to their semantic category. This human-
        # readable convention is possible because of how these meshes are generated and
        # stored in our internal systems.
        #
        # For "clutter" meshes, it is not straightforward to organize them by semantic
        # category, so instead we organize them according to their globally unique ID.
        #

        #
        # Individual assets should be named using a prefix that indicates the asset type
        # (e.g., "M_" for materials, "MI_" for material instances, "SM_" for static meshes,
        # "T_" for textures, etc). See [1] for more specific guidelines.
        #
        # [1] https://docs.unrealengine.com/5.2/en-US/recommended-asset-naming-conventions-in-unreal-engine-projects
        #

        #
        # TODO: The "Collision Presets" option should be set to "BlockAll".
        # TODO: The "Customized Collision" option should be enabled.
        #

        static_mesh = component.get_editor_property("static_mesh")
        if static_mesh is None:
            spear.log("WARNING: Unexpected static mesh: ", get_debug_string_actor(actor), ".", get_debug_string_component(component))
            return

        static_mesh_path = pathlib.PurePosixPath(static_mesh.get_path_name()).parts
        if len(static_mesh_path) != 8:
            spear.log(
                "WARNING: Unexpected static mesh path: ",
                get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_static_mesh(static_mesh), ")")
            return
        if static_mesh_path[0:5] != ("/", "Game", "Scenes", level, "Meshes"):
            spear.log(
                "WARNING: Unexpected static mesh path: ",
                get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_static_mesh(static_mesh), ")")
            return
        if static_mesh_path[5] not in ["Base", "Clutter"]:
            spear.log(
                "WARNING: Unexpected static mesh path: ",
                get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_static_mesh(static_mesh), ")")
            return
        if not static_mesh_path[7].startswith("SM_"):
            spear.log(
                "WARNING: Unexpected static mesh path: ",
                get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_static_mesh(static_mesh), ")")
            return

        #
        # Each material is kept here in a directory named according to the material's
        # globally unique ID.
        #

        materials = component.get_editor_property("override_materials")
        for material in materials:
            if material is None:
                spear.log(
                    "WARNING: Unexpected material: ",
                    get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_material(material), ")")
                return

            material_path = pathlib.PurePosixPath(material.get_path_name()).parts
            if len(material_path) < 4:
                spear.log(
                    "WARNING: Unexpected material path: ",
                    get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_material(material), ")")
                return

            if material_path[0:5] != ("/", "Game", "Scenes", level, "Materials") and \
               material_path[0:3] != ("/", "Game", "Megascans") and \
               material_path[0:4] != ("/", "Game", "Kujiale", "Materials"):
                spear.log(
                    "WARNING: Unexpected material path: ",
                    get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_material(material), ")")
                return

            base_material = material.get_base_material()
            if base_material.get_name() == "M_BaseMaterial":
                if material_path[-1] != "M_BaseMaterial.M_BaseMaterial" and not material_path[-1].startswith("MI_"):
                    spear.log(
                        "WARNING: Unexpected material path: ",
                        get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_material(material), ")")
                    return

            if base_material.get_name() == "M_TranslucentMaterial":
                if material_path[-1] != "M_TranslucentMaterial.M_TranslucentMaterial" and not material_path[-1].startswith("MI_"):
                    spear.log(
                        "WARNING: Unexpected material path: ",
                        get_debug_string_actor(actor), ".", get_debug_string_component(component), " (", get_debug_string_material(material), ")")
                    return


def vector_to_array(vector):
    return np.array([vector.x, vector.y, vector.z])

def array_to_vector(array):
    return unreal.Vector(array[0], array[1], array[2])

# Unlike the default ActorComponent.get_parent_components() function, this function returns parent components in root-to-leaf order.
def get_parent_components(component):
    c = component
    parents = []
    while c.get_attach_parent() is not None:
        c = c.get_attach_parent()
        parents = [c] + parents
    return parents

def get_debug_string_static_mesh(static_mesh):
    return static_mesh.get_path_name()

def get_debug_string_material(material):
    return material.get_path_name()

def get_debug_string_component(component):
    return ".".join([ c.get_name() for c in list(component.get_parent_components())[::-1] ]) + "." + component.get_name()

def get_debug_string_type(type):
    return type.__name__

def get_debug_string_actor(actor):
    return str(actor.get_folder_path()) + posixpath.sep + actor.get_actor_label()


if __name__ == "__main__":

    # TODO: parse command-line args

    # TODO: get level name programmatically
    level = "kujiale_0000"

    validate_level(level)
    validate_content_browser(level)
    validate_outliner(level)
