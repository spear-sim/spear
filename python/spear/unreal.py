#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import posixpath
import unreal

editor_actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)


def find_actors(actor_class=None):
    actors = editor_actor_subsystem.get_all_level_actors()
    actors = sorted(actors, key=lambda actor: get_stable_name_actor(actor))
    if actor_class is not None:
        actors = [ actor for actor in actors if actor.__class__.__name__ == actor_class ]
    return actors


def find_actor(stable_name):
    actors = [ actor for actor in find_actors() if get_stable_name_actor(actor) == stable_name ]
    assert len(actors) == 1
    return actors[0]


def find_components(actor, component_class=None):
    if actor.root_component is not None:
        components = [actor.root_component] + list(actor.root_component.get_children_components(include_all_descendants=True))
        if component_class is not None:
            components = [ component for component in components if component.__class__.__name__ == component_class ]
    else:
        components = []
    return components


def find_component(stable_name, actor=None):
    if actor is None:
        actor_stable_name, component_stable_name = stable_name.split(":")
        actor = find_actor(actor_stable_name)
    else:
        component_stable_name = stable_name
    components = [ component for component in find_components(actor) if get_stable_name_component(component) == component_stable_name ]
    assert len(components) == 1
    return components[0]


def get_stable_name_actor(actor):
    folder_path = actor.get_folder_path()
    if folder_path.is_none():
        return actor.get_actor_label()
    else:
        return str(folder_path) + posixpath.sep + actor.get_actor_label()


def get_stable_name_component(component, include_actor=False):

    if include_actor:
        actor_prefix_str = get_stable_name_actor(component.get_owner()) + ":"
    else:
        actor_prefix_str = ""

    # reverse to get parent components in root-to-leaf order
    component_names = [ component.get_name() for component in list(component.get_parent_components())[::-1] ] + [component.get_name()]
    component_names_str = ".".join(component_names)

    return actor_prefix_str + component_names_str
