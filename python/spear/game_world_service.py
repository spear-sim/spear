#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

from enum import Enum
import json

class GameWorldService():

    class PropertyDesc():
        property_
        value_ptr_

    # should match the EIncludeSuperFlag::Type enum in Engine\Source\Runtime\CoreUObject\Public\UObject\Class.h
    class EIncludeSuperFlag(Enum):
        ExcludeSuperFlag = 0
        IncludeSuperFlag = 1

    # should match the ELoadFlags class in Engine\Source\Runtime\CoreUObject\Public\UObject\ObjectMacros.h
    class ELoadFlags (Enum):
        LOAD_None						= 0x00000000,	#< No flags.
        LOAD_Async						= 0x00000001,	#< Loads the package using async loading path/ reader
        LOAD_NoWarn						= 0x00000002,	#< Don't display warning if load fails.
        LOAD_EditorOnly					= 0x00000004,	#< Load for editor-only purposes and by editor-only code
        LOAD_ResolvingDeferredExports	= 0x00000008,	#< Denotes that we should not defer export loading (as we're resolving them)
        LOAD_Verify						= 0x00000010,	#< Only verify existance; don't actually load.
        LOAD_NoVerify					= 0x00000080,	#< Don't verify imports yet.
        LOAD_IsVerifying				= 0x00000100,	#< Is verifying imports
        LOAD_SkipLoadImportedPackages	= 0x00000200,	#< Assume that all import packages are already loaded and don't call LoadPackage when creating imports 
        LOAD_RegenerateBulkDataGuids	= 0x00000400,	#< BulkData identifiers should be regenerated as they are loaded 
        LOAD_DisableDependencyPreloading = 0x00001000,	#< Bypass dependency preloading system
        LOAD_Quiet						= 0x00002000,	#< No log warnings.
        LOAD_FindIfFail					= 0x00004000,	#< Tries FindObject if a linker cannot be obtained (e.g. package is currently being compiled)
        LOAD_MemoryReader				= 0x00008000,	#< Loads the file into memory and serializes from there.
        LOAD_NoRedirects				= 0x00010000,	#< Never follow redirects when loading objects; redirected loads will fail
        LOAD_ForDiff					= 0x00020000,	#< Loading for diffing in the editor
        LOAD_PackageForPIE				= 0x00080000,	#< This package is being loaded for PIE, it must be flagged as such immediately
        LOAD_DeferDependencyLoads       = 0x00100000,	#< Do not load external (blueprint) dependencies (instead, track them for deferred loading)
        LOAD_ForFileDiff				= 0x00200000,	#< Load the package (not for diffing in the editor), instead verify at the two packages serialized output are the same, if they are not then debug break so that you can get the callstack and object information
        LOAD_DisableCompileOnLoad		= 0x00400000,	#< Prevent this load call from running compile on load for the loaded blueprint (intentionally not recursive, dependencies will still compile on load)
        LOAD_DisableEngineVersionChecks = 0x00800000,	#< Prevent this load call from running engine version checks

    class EObjectFlags(Enum):
	# Do not add new flags unless they truly belong here. There are alternatives.
	# if you change any the bit of any of the RF_Load flags, then you will need legacy serialization
	RF_NoFlags					= 0x00000000,	#< No flags, used to avoid a cast

	# This first group of flags mostly has to do with what kind of object it is. Other than transient, these are the persistent object flags.
	# The garbage collector also tends to look at these.
	RF_Public					=0x00000001,	#< Object is visible outside its package.
	RF_Standalone				=0x00000002,	#< Keep object around for editing even if unreferenced.
	RF_MarkAsNative				=0x00000004,	#< Object (UField) will be marked as native on construction (DO NOT USE THIS FLAG in HasAnyFlags() etc)
	RF_Transactional			=0x00000008,	#< Object is transactional.
	RF_ClassDefaultObject		=0x00000010,	#< This object is its class's default object
	RF_ArchetypeObject			=0x00000020,	#< This object is a template for another object - treat like a class default object
	RF_Transient				=0x00000040,	#< Don't save object.

	# This group of flags is primarily concerned with garbage collection.
	RF_MarkAsRootSet			=0x00000080,	#< Object will be marked as root set on construction and not be garbage collected, even if unreferenced (DO NOT USE THIS FLAG in HasAnyFlags() etc)
	RF_TagGarbageTemp			=0x00000100,	#< This is a temp user flag for various utilities that need to use the garbage collector. The garbage collector itself does not interpret it.

	# The group of flags tracks the stages of the lifetime of a uobject
	RF_NeedInitialization		=0x00000200,	#< This object has not completed its initialization process. Cleared when ~FObjectInitializer completes
	RF_NeedLoad					=0x00000400,	#< During load, indicates object needs loading.
	RF_KeepForCooker			=0x00000800,	#< Keep this object during garbage collection because it's still being used by the cooker
	RF_NeedPostLoad				=0x00001000,	#< Object needs to be postloaded.
	RF_NeedPostLoadSubobjects	=0x00002000,	#< During load, indicates that the object still needs to instance subobjects and fixup serialized component references
	RF_NewerVersionExists		=0x00004000,	#< Object has been consigned to oblivion due to its owner package being reloaded, and a newer version currently exists
	RF_BeginDestroyed			=0x00008000,	#< BeginDestroy has been called on the object.
	RF_FinishDestroyed			=0x00010000,	#< FinishDestroy has been called on the object.

	# Misc. Flags
	RF_BeingRegenerated			=0x00020000,	#< Flagged on UObjects that are used to create UClasses (e.g. Blueprints) while they are regenerating their UClass on load (See FLinkerLoad::CreateExport()), as well as UClass objects in the midst of being created
	RF_DefaultSubObject			=0x00040000,	#< Flagged on subobjects that are defaults
	RF_WasLoaded				=0x00080000,	#< Flagged on UObjects that were loaded
	RF_TextExportTransient		=0x00100000,	#< Do not export object to text form (e.g. copy/paste). Generally used for sub-objects that can be regenerated from data in their parent object.
	RF_LoadCompleted			=0x00200000,	#< Object has been completely serialized by linkerload at least once. DO NOT USE THIS FLAG, It should be replaced with RF_WasLoaded.
	RF_InheritableComponentTemplate = 0x00400000, #< Archetype of the object can be in its super class
	RF_DuplicateTransient		=0x00800000,	#< Object should not be included in any type of duplication (copy/paste, binary duplication, etc.)
	RF_StrongRefOnFrame			=0x01000000,	#< References to this object from persistent function frame are handled as strong ones.
	RF_NonPIEDuplicateTransient	=0x02000000,	#< Object should not be included for duplication unless it's being duplicated for a PIE session
	RF_WillBeLoaded				=0x08000000,	#< This object was constructed during load and will be loaded shortly
	RF_HasExternalPackage		=0x10000000,	#< This object has an external package assigned and should look it up when getting the outermost package

	# RF_Garbage and RF_PendingKill are mirrored in EInternalObjectFlags because checking the internal flags is much faster for the Garbage Collector
	# while checking the object flags is much faster outside of it where the Object pointer is already available and most likely cached.
	# RF_PendingKill is mirrored in EInternalObjectFlags because checking the internal flags is much faster for the Garbage Collector
	# while checking the object flags is much faster outside of it where the Object pointer is already available and most likely cached.
	RF_AllocatedInSharedPage	=0x80000000,	///< Allocated from a ref-counted page shared with other UObjects

    def __init__(self, rpc_client):
        self._rpc_client = rpc_client

    def get_world_name(self):
        return self._rpc_client.call("game_world_service.get_world_name")

    def open_level(self, level_name):
        self._rpc_client.call("game_world_service.open_level", level_name)

    def set_game_paused(self, paused):
        self._rpc_client.call("game_world_service.set_game_paused", paused)

    def find_actors(self):
        return self._rpc_client.call("game_world_service.find_actors")

    def find_actors_as_map(self):
        return self._rpc_client.call("game_world_service.find_actors_as_map")    

    def get_components(self, actor):
        return self._rpc_client.call("game_world_service.get_components", actor)

    def get_components_as_map(self, actor):
        return self._rpc_client.call("game_world_service.get_components_as_map", actor)

    def get_children_components(self, actor, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components", actor, include_all_descendants)

    def get_children_components_as_map(self, actor, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_as_map", actor, include_all_descendants)

    def get_object_properties_as_string_from_uobject(self, uobject):
        return self._rpc_client.call("game_world_service.get_object_properties_as_string_from_uobject", uobject)

    def get_object_properties_as_string_from_ustruct(self, value_ptr, ustruct):
        return self._rpc_client.call("game_world_service.get_object_properties_as_string_from_ustruct", value_ptr, ustruct)

    def set_object_properties_from_string_for_uobject(self, uobject, string):
        self._rpc_client.call("game_world_service.set_object_properties_from_string_for_uobject", uobject, string)

    def set_object_properties_from_string_for_ustruct(self, value_ptr, ustruct, string):
        self._rpc_client.call("game_world_service.set_object_properties_from_string_for_ustruct", value_ptr, ustruct, string)

    def find_property_by_name_on_object(self, uobject, name):
        return self._rpc_client.call("game_world_service.find_property_by_name_on_uobject", uobject, name)

    def find_property_by_name_on_struct(self, value_ptr, ustruct, name):
        return self._rpc_client.call("game_world_service.find_property_by_name_on_ustruct", value_ptr, ustruct, name)

    def get_property_value_as_string(self, property_desc):
        return self._rpc_client.call("game_world_service.get_property_value_as_string", property_desc)

    def set_property_value_from_string(self, property_desc, string):
        return self._rpc_client.call("game_world_service.set_property_value_from_string", property_desc, string)

    def find_function_by_name(self, uclass, name, include_super_flag):
        return self._rpc_client.call("game_world_service.find_function_by_name", uclass, name, include_super_flag)

    def call_function(self, uobject, ufunction, **kwargs):
        func_args = {}
        for key, value in kwargs.items():
            if isinstance(value, zip):
                func_args[key] = json.dumps(dict(value))
            elif isinstance(value, (dict, list)):
                func_args[key] = json.dumps(value)
            elif isinstance(value, (bool, float, int)):
                func_args[key] = str(value)
            elif isinstance(value, str):
                func_args[key] = value
            else:
                assert False
        return self._rpc_client.call("game_world_service.call_function", uobject, ufunction, func_args)

    def find_special_struct_by_name(self, name):
        return self._rpc_client.call("game_world_service.find_special_struct_by_name", name)

    def has_stable_name(self, actor):
        return self._rpc_client.call("game_world_service.has_stable_name", actor)

    def get_stable_name_for_actor(self, actor):
        return self._rpc_client.call("game_world_service.get_stable_name_for_actor", actor)

    def get_stable_name_for_actor_component(self, component, include_actor_name):
        return self._rpc_client.call("game_world_service.get_stable_name_for_actor_component", component, include_actor_name)

    def get_stable_name_for_scene_component(self, component, include_actor_name):
        return self._rpc_client.call("game_world_service.get_stable_name_for_scene_component", component, include_actor_name)

    def get_actor_tags(self, actor):
        return self._rpc_client.call("game_world_service.get_actor_tags", actor)

    def get_component_tags(self, actor):
        return self._rpc_client.call("game_world_service.get_component_tags", actor)

    def get_class(self, uobject):
        return self._rpc_client.call("game_world_service.get_class", uobject)

    def get_static_class(self, class_name):
        return self._rpc_client.call("game_world_service.get_static_class", class_name)

    def create_component_outside_owner_constructor(class_name, owner, name):
        return self._rpc_client.call("game_world_service.create_component_outside_owner_constructor", class_name, owner, name)

    def create_scene_component_outside_owner_constructor_from_actor(class_name, actor, name):
        return self._rpc_client.call("game_world_service.create_scene_component_outside_owner_constructor_from_actor", class_name, actor, name)

    def create_scene_component_outside_owner_constructor_from_object(class_name, owner, parent, name):
        return self._rpc_client.call("game_world_service.create_scene_component_outside_owner_constructor_from_object", class_name, owner, parent, name)

    def create_scene_component_outside_owner_constructor_from_component(class_name, owner, name):
        return self._rpc_client.call("game_world_service.create_scene_component_outside_owner_constructor_from_component", class_name, owner, name)

    def new_object(class_name, outer, name, flags, uobject_template, copy_transients_from_class_defaults, in_instance_graph):
        return self._rpc_client.call("game_world_service.new_object", class_name, outer, name, flags, uobject_template, copy_transients_from_class_defaults, in_instance_graph)

    def load_object(class_name, outer, name, filename, load_flags, sandbox, instancing_context):
        return self._rpc_client.call("game_world_service.load_object", class_name, outer, name, filename, load_flags, sandbox, instancing_context)

    def find_actors_by_name(class_name, names, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.find_actors_by_name", class_name, names, return_null_if_not_found)

    def find_actors_by_tag(class_name, tag):
        return self._rpc_client.call("game_world_service.find_actors_by_tag", class_name, tag)

    def find_actors_by_tag_any(class_name, tags):
        return self._rpc_client.call("game_world_service.find_actors_by_tag_any", class_name, tags)

    def find_actors_by_tag_all(class_name, tags):
        return self._rpc_client.call("game_world_service.find_actors_by_tag_all", class_name, tags)

    def find_actors_by_type(class_name):
        return self._rpc_client.call("game_world_service.find_actors_by_type", class_name)

    def find_actors_by_name_as_map(class_name, names, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.find_actors_by_name_as_map", class_name, names, return_null_if_not_found)

    def find_actors_by_tag_as_map(class_name, tag):
        return self._rpc_client.call("game_world_service.find_actors_by_tag_as_map", class_name, tag)

    def find_actors_by_tag_any_as_map(class_name, tags):
        return self._rpc_client.call("game_world_service.find_actors_by_tag_any_as_map", class_name, tags)

    def find_actors_by_tag_all_as_map(class_name, tags):
        return self._rpc_client.call("game_world_service.find_actors_by_tag_all_as_map", class_name, tags)

    def find_actors_by_type_as_map(class_name):
        return self._rpc_client.call("game_world_service.find_actors_by_type_as_map", class_name)

    def find_actor_by_name(class_name, name, assert_if_not_found):
        return self._rpc_client.call("game_world_service.find_actor_by_name", class_name, name, assert_if_not_found)

    def find_actor_by_tag(class_name, tag, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.find_actor_by_tag", class_name, tag, assert_if_not_found, assert_if_multiple_found)

    def find_actor_by_tag_any(class_name, tags, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.find_actor_by_tag_any", class_name, tags, assert_if_not_found, assert_if_multiple_found)

    def find_actor_by_tag_all(class_name, tags, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.find_actor_by_tag_all", class_name, tags, assert_if_not_found, assert_if_multiple_found)

    def find_actor_by_type(class_name, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.find_actor_by_type", class_name, assert_if_not_found, assert_if_multiple_found)

    def get_components_by_name(class_name, actor, names, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_components_by_name", class_name, actor, names, return_null_if_not_found)

    def get_components_by_tag(class_name, actor, tag):
        return self._rpc_client.call("game_world_service.get_components_by_tag", class_name, actor, tag)

    def get_components_by_tag_any(class_name, actor, tags):
        return self._rpc_client.call("game_world_service.get_components_by_tag_any", class_name, actor, tags)

    def get_components_by_tag_all(class_name, actor, tags):
        return self._rpc_client.call("game_world_service.get_components_by_tag_all", class_name, actor, tags)

    def get_components_by_type(class_name, actor):
        return self._rpc_client.call("game_world_service.get_components_by_type", class_name, actor)

    def get_components_by_name_as_map(class_name, actor, names, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_components_by_name_as_map", class_name, actor, names, return_null_if_not_found)

    def get_components_by_tag_as_map(class_name, actor, tag):
        return self._rpc_client.call("game_world_service.get_components_by_tag_as_map", class_name, actor, tag)

    def get_components_by_tag_any_as_map(class_name, actor, tags):
        return self._rpc_client.call("game_world_service.get_components_by_tag_any_as_map", class_name, actor, tags)

    def get_components_by_tag_all_as_map(class_name, actor, tags):
        return self._rpc_client.call("game_world_service.get_components_by_tag_all_as_map", class_name, actor, tags)

    def get_components_by_type_as_map(class_name, actor):
        return self._rpc_client.call("game_world_service.get_components_by_type_as_map", class_name, actor)

    def get_component_by_name(class_name, actor, name, assert_if_not_found):
        return self._rpc_client.call("game_world_service.get_component_by_name", class_name, actor, name, assert_if_not_found)

    def get_component_by_tag(class_name, actor, tag, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_component_by_tag", class_name, actor, tag, assert_if_not_found, assert_if_multiple_found)

    def get_component_by_tag_any(class_name, actor, tags, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_component_by_tag_any", class_name, actor, tags, assert_if_not_found, assert_if_multiple_found)

    def get_component_by_tag_all(class_name, actor, tags, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_component_by_tag_all", class_name, actor, tags, assert_if_not_found, assert_if_multiple_found)

    def get_component_by_type(class_name, actor, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_component_by_type", class_name, actor, assert_if_not_found, assert_if_multiple_found)

    def get_children_components_by_name_from_actor(class_name,  parent, names, include_all_descendants, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_children_components_by_name_from_actor", class_name,  parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_from_actor(class_name, parent, tag, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_from_actor", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_from_actor(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_any_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_from_actor(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_all_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_from_actor(class_name, parent, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_type_from_actor", class_name, parent, include_all_descendants)

    def get_children_components_by_name_as_map_from_actor(class_name, parent, names, include_all_descendants, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_children_components_by_name_as_map_from_actor", class_name, parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_as_map_from_actor(class_name, parent, tag, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_as_map_from_actor", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_as_map_from_actor(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_any_as_map_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_as_map_from_actor(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_all_as_map_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_as_map_from_actor(class_name, parent, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_type_as_map_from_actor", class_name, parent, include_all_descendants)

    def get_child_component_by_name_from_actor(class_name, parent, name, include_all_descendants, assert_if_not_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_name_from_actor", class_name, parent, name, include_all_descendants, assert_if_not_found)

    def get_child_component_by_tag_from_actor(class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_from_actor", class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_any_from_actor(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_any_from_actor", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_all_from_actor(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_all_from_actor", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_type_from_actor(class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_type_from_actor", class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_children_components_by_name_from_scene_component(class_name, parent, names, include_all_descendants, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_children_components_by_name_from_scene_component", class_name, parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_from_scene_component(class_name, parent, tag, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_from_scene_component", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_from_scene_component(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_any_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_from_scene_component(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_all_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_from_scene_component(class_name, parent, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_type_from_scene_component", class_name, parent, include_all_descendants)

    def get_children_components_by_name_as_map_from_scene_component(class_name, parent, names, include_all_descendants, return_null_if_not_found):
        return self._rpc_client.call("game_world_service.get_children_components_by_name_as_map_from_scene_component", class_name, parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_as_map_from_scene_component(class_name, parent, tag, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_as_map_from_scene_component", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_as_map_from_scene_component(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_any_as_map_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_as_map_from_scene_component(class_name, parent, tags, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_tag_all_as_map_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_as_map_from_scene_component(class_name, parent, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_by_type_as_map_from_scene_component", class_name, parent, include_all_descendants)

    def get_child_component_by_name_from_scene_component(class_name, parent, name, include_all_descendants, assert_if_not_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_name_from_scene_component", class_name, parent, name, include_all_descendants, assert_if_not_found)

    def get_child_component_by_tag_from_scene_component(class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_from_scene_component", class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_any_from_scene_component(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_any_from_scene_component", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_all_from_scene_component(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_tag_all_from_scene_component", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_type_from_scene_component(class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found):
        return self._rpc_client.call("game_world_service.get_child_component_by_type_from_scene_component", class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found)
