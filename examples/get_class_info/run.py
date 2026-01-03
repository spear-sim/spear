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

        # get UKismetSystemLibrary
        kismet_system_library = game.get_unreal_object(uclass="UKismetSystemLibrary")

        # spawn object
        bp_axes_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")
        bp_axes = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})
        bp_axes.print_debug_info()

        spear.log("Counting the number of functions and properties exposed to the Unreal Engine property system...")

        total_num_functions = 0
        total_num_properties = 0
        uclasses = {}

        uobject_uclass = game.unreal_service.get_static_class(uclass="UObject")
        uobject_meta_uclass = game.unreal_service.get_class(uobject=uobject_uclass)
        spear.log("    Type: ", game.unreal_service.get_type_for_class_as_string(uclass=uobject_uclass))
        spear.log("    Meta type: ", game.unreal_service.get_type_for_class_as_string(uclass=uobject_meta_uclass))

        num_functions = len(game.unreal_service.find_functions(uclass=uobject_uclass, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
        num_properties = len(game.unreal_service.find_properties_for_class(uclass=uobject_uclass, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
        spear.log("        Class: UObject (", num_functions, " functions, ", num_properties, " properties)")
        total_num_functions = total_num_functions + num_functions
        total_num_properties = total_num_properties + num_properties

        uclasses = game.unreal_service.get_derived_classes_as_dict(uclass=uobject_uclass)
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

        uobject_uclass = game.unreal_service.get_static_class(uclass="UObject")
        uobject_meta_uclass = game.unreal_service.get_class(uobject=uobject_uclass)
        # spear.log("    Meta type: ", game.unreal_service.get_type_for_class_as_string(uclass=uobject_meta_uclass))
        # spear.log("    Type: ", game.unreal_service.get_type_for_class_as_string(uclass=uobject_uclass))

        function_flags = ["FUNC_BlueprintCallable", "FUNC_BlueprintPure"]
        property_flags = ["CPF_BlueprintVisible", "CPF_BlueprintReadOnly", "CPF_BlueprintAssignable", "CPF_Edit"]

        num_functions = len(game.unreal_service.find_functions_by_flags_any(uclass=uobject_uclass, function_flags=function_flags, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
        num_properties = len(game.unreal_service.find_properties_for_class_by_flags_any(uclass=uobject_uclass, property_flags=property_flags, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
        # spear.log("    Class: UObject (", num_functions, " functions, ", num_properties, " properties)")
        total_num_functions = total_num_functions + num_functions
        total_num_properties = total_num_properties + num_properties

        uclasses = game.unreal_service.get_derived_classes_as_dict(uclass=uobject_uclass)
        # spear.log("    Number of classes that derive from UObject: ", len(uclasses))

        for uclass_name, uclass in uclasses.items():
            num_functions = len(game.unreal_service.find_functions_by_flags_any(uclass=uclass, function_flags=function_flags, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
            num_properties = len(game.unreal_service.find_properties_for_class_by_flags_any(uclass=uclass, property_flags=property_flags, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
            # spear.log("        Class: ", uclass_name, " (", num_functions, " functions, ", num_properties, " properties)")
            total_num_functions = total_num_functions + num_functions
            total_num_properties = total_num_properties + num_properties

        ustructs = game.unreal_service.find_static_structs_as_dict()
        # spear.log("    Number of structs that are outside the UObject class hierarchy: ", len(ustructs))

        for ustruct_name, ustruct in ustructs.items():
            num_properties = len(game.unreal_service.find_properties_for_struct_by_flags_any(ustruct=ustruct, property_flags=property_flags, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
            # spear.log("        Struct: ", ustruct_name, " (", num_properties, " properties)")
            total_num_properties = total_num_properties + num_properties

        spear.log("    Total function count: ", total_num_functions)
        spear.log("    Total property count: ", total_num_properties)

        spear.log("Counting the number of functions available on AActor classes...")

        total_num_functions = 0
        uclasses = {}

        actor_uclass = game.unreal_service.get_static_class(uclass="AActor")
        actor_meta_uclass = game.unreal_service.get_class(uobject=actor_uclass)
        spear.log("    Type: ", game.unreal_service.get_type_for_class_as_string(uclass=actor_uclass))
        spear.log("    Meta type: ", game.unreal_service.get_type_for_class_as_string(uclass=actor_meta_uclass))

        num_functions = len(game.unreal_service.find_functions(uclass=actor_uclass, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
        spear.log("        Class: AActor (", num_functions, " functions)")
        total_num_functions = total_num_functions + num_functions

        uclasses = game.unreal_service.get_derived_classes_as_dict(uclass=actor_uclass)
        spear.log("    Number of classes that derive from AActor: ", len(uclasses))

        for uclass_name, uclass in uclasses.items():
            num_functions = len(game.unreal_service.find_functions(uclass=uclass, field_iteration_flags=["IncludeDeprecated"])) # exclude base classes
            spear.log("        Class: ", uclass_name, " (", num_functions, " functions)")
            total_num_functions = total_num_functions + num_functions

        spear.log("    Total function count: ", total_num_functions)

    with instance.end_frame():
        pass

    spear.log("Done.")
