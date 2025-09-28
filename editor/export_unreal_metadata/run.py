#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import os
import pandas as pd
import posixpath
import spear
import spear.utils.editor_utils
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--export_dir", required=True)
args = parser.parse_args()

unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
level_editor_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)

editor_world_name = unreal_editor_subsystem.get_editor_world().get_name()

editor_properties_csv_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "editor_properties.csv"))
df_editor_properties = pd.read_csv(editor_properties_csv_file)


def process_scene():
    
    spear.log("Processing scene: " + editor_world_name)

    actors = spear.utils.editor_utils.find_actors()
    actors = { spear.utils.editor_utils.get_stable_name_for_actor(actor=actor): get_actor_desc(actor) for actor in actors }

    unreal_metadata_dir = os.path.realpath(os.path.join(args.export_dir, "scenes", editor_world_name, "unreal_metadata"))
    actors_json_file = os.path.realpath(os.path.join(unreal_metadata_dir, "scene.json"))
    spear.log("Writing JSON file: " + actors_json_file)
    os.makedirs(unreal_metadata_dir, exist_ok=True)
    with open(actors_json_file, "w") as f:
        json.dump(actors, f, indent=4, sort_keys=True)

    spear.log("Done.")

def get_actor_desc(actor):
    actor_name = spear.utils.editor_utils.get_stable_name_for_actor(actor=actor)
    spear.log("Processing actor: ", actor_name)

    if actor.root_component is not None:
        components_in_hierarchy = [actor.root_component] + list(actor.root_component.get_children_components(include_all_descendants=True))
    else:
        components_in_hierarchy = []
    components_in_hierarchy_names = [ spear.utils.editor_utils.get_stable_name_for_component(c) for c in components_in_hierarchy ]

    other_components = actor.get_components_by_class(unreal.ActorComponent)
    other_components = [ c for c in other_components if spear.utils.editor_utils.get_stable_name_for_component(c) not in components_in_hierarchy_names ]

    return {
        "class": actor.__class__.__name__,
        "debug_info": {"str": str(actor)},
        "editor_properties": get_editor_property_descs(actor),
        "name": actor_name,
        "other_components": { spear.utils.editor_utils.get_stable_name_for_component(c): get_component_desc(c) for c in other_components },
        "root_component": get_component_desc(actor.get_editor_property("root_component"))}

def get_component_desc(component):

    if component is None:
        return None

    if "get_children_components" in dir(component):
        children_components = { spear.utils.editor_utils.get_stable_name_for_component(c): get_component_desc(c) for c in component.get_children_components(include_all_descendants=False) }
    else:
        children_components = None

    component_desc = {
        "children_components": children_components,
        "class": component.__class__.__name__,
        "debug_info": {"str": str(component)},
        "editor_properties": get_editor_property_descs(component),
        "name": spear.utils.editor_utils.get_stable_name_for_component(component=component),
        "pipeline_info": {},
        "unreal_name": component.get_name()}

    if isinstance(component, unreal.SceneComponent):
        component_desc["debug_info"]["world_transform"] = get_editor_property_desc(component.get_world_transform().to_matrix())

    return component_desc

def get_editor_property_descs(uobject):

    assert "get_editor_property" in dir(uobject)

    # The Unreal Python interface does not provide a mechanism to iterate over editor properties directly,
    # so we use the fact that most editor properties are exposed as Python attributes, and iterate over
    # attributes instead. For each attribute, we use a guess-and-check strategy to see if it is an editor
    # property. We call uobject.get_editor_property(...) and if doing so doesn't throw an exception, we
    # assume that the attribute is a valid editor property. For each editor property that we find, if the
    # value of the editor property is itself a uobject, then we recurse, otherwise we store its value in
    # our output dict. To obtain any remaining editor properties that are not exposed as Python attributes,
    # we maintain a CSV file with a list of editor properties for various classes of interest. We populate
    # the CSV file by manually copying and pasting from the Unreal documentation.

    # Get all Python attributes, and all editor properties from our CSV file.
    uobject_class_names = [ c.__name__ for c in uobject.__class__.mro() ] # derived-to-base order
    editor_property_candidate_names = set(dir(uobject))
    for uobject_class_name in uobject_class_names:
        editor_property_candidate_names = editor_property_candidate_names | set(df_editor_properties.loc[df_editor_properties["class"] == uobject_class_name]["editor_property"])

    # Guess-and-check all editor property candidates.
    editor_property_descs = {}
    for editor_property_candidate_name in editor_property_candidate_names:
        try:
            editor_property = uobject.get_editor_property(editor_property_candidate_name)
            editor_property_descs[editor_property_candidate_name] = get_editor_property_desc(editor_property)
        except:
            pass

    return editor_property_descs

def get_editor_property_desc(editor_property):

    # If the editor property is None, then return None, because we want it to be serialized as null
    # rather than "None"
    if editor_property is None:
        return None

    # Otherwise, if the editor property is an Actor or ActorComponent, then do not return any editor
    # properties to avoid an infinite recursion. If users want to obtain the editor properties for an
    # Actor or ActorComponent, they must call get_editor_property_descs(...).

    elif isinstance(editor_property, unreal.Actor):
        return {
            "class": editor_property.__class__.__name__,
            "debug_info": {"str": str(editor_property)},
            "name": spear.utils.editor_utils.get_stable_name_for_actor(actor=editor_property)}

    elif isinstance(editor_property, unreal.ActorComponent):
        return {
            "class": editor_property.__class__.__name__,
            "debug_info": {"str": str(editor_property)},
            "name": spear.utils.editor_utils.get_stable_name_for_component(component=editor_property)}

    # Otherwise, if the editor property is a StaticMesh, then recurse via get_editor_property_descs(...).
    elif isinstance(editor_property, unreal.StaticMesh):
        return {
            "class": editor_property.__class__.__name__,
            "debug_info": {"str": str(editor_property)},
            "editor_properties": get_editor_property_descs(editor_property),
            "path": editor_property.get_path_name()}

    # Otherwise, if the editor property is an Unreal object or struct, then recurse via get_editor_property_descs(...).
    elif isinstance(editor_property, unreal.Object) or isinstance(editor_property, unreal.StructBase):
        return {
            "class": editor_property.__class__.__name__,
            "debug_info": {"str": str(editor_property)},
            "editor_properties": get_editor_property_descs(editor_property)}

    # Otherwise, if the editor property is an Unreal array, then recurse via get_editor_property_desc(...).
    elif isinstance(editor_property, unreal.Array):
        return [ get_editor_property_desc(editor_property_array_entry) for editor_property_array_entry in editor_property ]

    # Otherwise, if the editor property value is serializable as JSON, then return the object,
    # otherwise return the string representation of the object.
    else:
        try:
            json.dumps(editor_property)
            return editor_property
        except:
            return str(editor_property)


if __name__ == "__main__":
    process_scene()
