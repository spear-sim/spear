#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import pprint
import spear


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()
    
    with instance.begin_frame():

        kismet_system_library_static_class = game.unreal_service.get_static_class(class_name="UKismetSystemLibrary")
        kismet_system_library_default_object = game.unreal_service.get_default_object(uclass=kismet_system_library_static_class, create_if_needed=False)
        is_object_of_soft_class_func = game.unreal_service.find_function_by_name(uclass=kismet_system_library_static_class, function_name="IsObjectOfSoftClass")

        # spawn object
        bp_axes_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")
        bp_axes = game.unreal_service.spawn_actor_from_class(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})

        spear.log("Printing info for actor: ", game.unreal_service.try_get_stable_name_for_actor(bp_axes))

        spear.log("    Non-scene components: ")
        components = game.unreal_service.get_components_as_dict(bp_axes)
        for component_name, component in components.items():
            args = {"Object": spear.to_ptr(component), "SoftClass": "/Script/Engine.SceneComponent"}
            return_values = game.unreal_service.call_function(uobject=kismet_system_library_default_object, ufunction=is_object_of_soft_class_func, args=args)
            return_value = return_values["ReturnValue"]
            if not return_value:
                component_class = game.unreal_service.get_class(uobject=component)
                spear.log("        ", component_name, " (", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=component_class), ")")

        spear.log("    Scene components: ")
        components = game.unreal_service.get_children_components_from_actor_as_dict(parent=bp_axes)
        for component_name, component in components.items():
            component_class = game.unreal_service.get_class(uobject=component)
            spear.log("        ", component_name, " (", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=component_class), ")")

        bp_axes_uclass = game.unreal_service.get_class(uobject=bp_axes)
        bp_axes_meta_uclass = game.unreal_service.get_class(uobject=bp_axes_uclass)

        spear.log("    Meta type: ", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=bp_axes_meta_uclass))
        spear.log("    Target type: ", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=bp_axes_uclass))

        spear.log("    C++ type hierarchy: ")
        current_uclass = bp_axes_uclass
        while current_uclass != 0:
            spear.log("        ", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=current_uclass))
            current_uclass = game.unreal_service.get_super_class(uclass=current_uclass)

        spear.log("    Functions: ")
        current_uclass = bp_axes_uclass
        while current_uclass != 0:
            spear.log("        Functions for type: ", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=current_uclass))
            ufunctions = game.unreal_service.find_functions_as_dict(uclass=current_uclass, field_iteration_flags=["IncludeDeprecated"]) # exclude base classes
            for ufunction_name, ufunction in ufunctions.items():
                spear.log("            Function: ", ufunction_name)
                props = game.unreal_service.find_properties_for_function_as_dict(ufunction=ufunction)
                for prop_name, prop in props.items():
                    property_flags = game.unreal_service.get_property_flags(prop=prop)
                    assert "CPF_Parm" in property_flags
                    if "CPF_ReturnParm" not in property_flags:
                        spear.log("                Argument: ", prop_name, " (", game.unreal_service.get_cpp_type_for_property_as_string(prop=prop), ")")
                for prop_name, prop in props.items():
                    property_flags = game.unreal_service.get_property_flags(prop=prop)
                    assert "CPF_Parm" in property_flags
                    if "CPF_ReturnParm" in property_flags:
                        spear.log("                Return value: ", prop_name, " (", game.unreal_service.get_cpp_type_for_property_as_string(prop=prop), ")")
            current_uclass = game.unreal_service.get_super_class(uclass=current_uclass)

        spear.log("    Properties: ")
        current_uclass = bp_axes_uclass
        while current_uclass != 0:
            spear.log("        Properties for type: ", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=current_uclass))
            props = game.unreal_service.find_properties_for_struct_as_dict(ustruct=current_uclass, field_iteration_flags=["IncludeDeprecated"]) # exclude base classes
            for name, prop in props.items():
                spear.log("            Property: ", name, " (", game.unreal_service.get_cpp_type_for_property_as_string(prop=prop), ")")
            current_uclass = game.unreal_service.get_super_class(uclass=current_uclass)

        spear.log("Counting the number of functions and properties exposed to the Unreal Engine property system...")

        total_num_functions = 0
        total_num_properties = 0
        uclasses = {}

        uobject_static_class = game.unreal_service.get_static_class(class_name="UObject")
        uobject_static_class_uclass = game.unreal_service.get_class(uobject=uobject_static_class)
        spear.log("    Meta type: ", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=uobject_static_class_uclass))
        spear.log("    Target type: ", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=uobject_static_class))

        num_functions = len(game.unreal_service.find_functions(uclass=uobject_static_class, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
        num_properties = len(game.unreal_service.find_properties_for_struct(ustruct=uobject_static_class, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
        spear.log("    Class: UObject (", num_functions, " functions, ", num_properties, " properties)")
        total_num_functions = total_num_functions + num_functions
        total_num_properties = total_num_properties + num_properties

        uclasses = game.unreal_service.get_derived_classes_as_dict(uobject_static_class)
        spear.log("    Number of classes that derive from UObject: ", len(uclasses))

        for uclass_name, uclass in uclasses.items():
            num_functions = len(game.unreal_service.find_functions(uclass=uclass, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
            num_properties = len(game.unreal_service.find_properties_for_struct(ustruct=uclass, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
            spear.log("        Class: ", uclass_name, " (", num_functions, " functions, ", num_properties, " properties)")
            total_num_functions = total_num_functions + num_functions
            total_num_properties = total_num_properties + num_properties

        ustructs = game.unreal_service.find_static_structs_as_dict()
        spear.log("    Number of structs that are outside the UObject class hierarchy: ", len(ustructs))

        for ustruct_name, ustruct in ustructs.items():
            num_properties = len(game.unreal_service.find_properties_for_struct(ustruct=ustruct, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
            spear.log("        Struct: ", ustruct_name, " (", num_properties, " properties)")
            total_num_properties = total_num_properties + num_properties

        spear.log("    Total function count: ", total_num_functions)
        spear.log("    Total property count: ", total_num_properties)

        spear.log("Counting the number of functions and properties exposed to Unreal's editor-only Python library...")

        total_num_functions = 0
        total_num_properties = 0
        uclasses = {}

        uobject_static_class = game.unreal_service.get_static_class(class_name="UObject")
        uobject_static_class_uclass = game.unreal_service.get_class(uobject=uobject_static_class)
        # spear.log("    Meta type: ", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=uobject_static_class_uclass))
        # spear.log("    Target type: ", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=uobject_static_class))

        function_flags = ["FUNC_BlueprintCallable", "FUNC_BlueprintPure"]
        property_flags = ["CPF_BlueprintVisible", "CPF_BlueprintReadOnly", "CPF_BlueprintAssignable", "CPF_Edit"]

        num_functions = len(game.unreal_service.find_functions_by_flags_any(uclass=uobject_static_class, function_flags=function_flags, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
        num_properties = len(game.unreal_service.find_properties_for_struct_by_flags_any(ustruct=uobject_static_class, property_flags=property_flags, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
        # spear.log("    Class: UObject (", num_functions, " functions, ", num_properties, " properties)")
        total_num_functions = total_num_functions + num_functions
        total_num_properties = total_num_properties + num_properties

        uclasses = game.unreal_service.get_derived_classes_as_dict(uobject_static_class)
        spear.log("    Number of classes that derive from UObject: ", len(uclasses))

        for uclass_name, uclass in uclasses.items():
            num_functions = len(game.unreal_service.find_functions_by_flags_any(uclass=uclass, function_flags=function_flags, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
            num_properties = len(game.unreal_service.find_properties_for_struct_by_flags_any(ustruct=uclass, property_flags=property_flags, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
            # spear.log("        Class: ", uclass_name, " (", num_functions, " functions, ", num_properties, " properties)")
            total_num_functions = total_num_functions + num_functions
            total_num_properties = total_num_properties + num_properties

        ustructs = game.unreal_service.find_static_structs_as_dict()
        spear.log("    Number of structs that are outside the UObject class hierarchy: ", len(ustructs))

        for ustruct_name, ustruct in ustructs.items():
            num_properties = len(game.unreal_service.find_properties_for_struct_by_flags_any(ustruct=ustruct, property_flags=property_flags, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
            # spear.log("        Struct: ", ustruct_name, " (", num_properties, " properties)")
            total_num_properties = total_num_properties + num_properties

        spear.log("    Total function count: ", total_num_functions)
        spear.log("    Total property count: ", total_num_properties)

        spear.log("Counting the number of functions available through the AActor class...")

        total_num_functions = 0
        uclasses = {}

        uobject_static_class = game.unreal_service.get_static_class(class_name="AActor")
        uobject_static_class_uclass = game.unreal_service.get_class(uobject=uobject_static_class)
        # spear.log("    Meta type: ", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=uobject_static_class_uclass))
        # spear.log("    Target type: ", game.unreal_service.get_cpp_type_for_struct_as_string(ustruct=uobject_static_class))

        num_functions = len(game.unreal_service.find_functions(uclass=uobject_static_class, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
        # spear.log("    Class: AActor (", num_functions, " functions)")
        total_num_functions = total_num_functions + num_functions

        uclasses = game.unreal_service.get_derived_classes_as_dict(uobject_static_class)
        spear.log("    Number of classes that derive from AActor: ", len(uclasses))

        for uclass_name, uclass in uclasses.items():
            num_functions = len(game.unreal_service.find_functions(uclass=uclass, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
            # spear.log("            Class: ", uclass_name, " (", num_functions, " functions)")
            total_num_functions = total_num_functions + num_functions

        spear.log("    Total function count: ", total_num_functions)

    with instance.end_frame():
        pass

    spear.log("Done.")
