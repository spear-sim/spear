//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool;

public class Vehicle : SpModuleRules
{
    public Vehicle(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        // As a matter of convenience, it is possible to place most Unreal module dependencies in SpModuleRules without
        // needing to make any changes to our uplugin files. The ChaosVehicles module is different. If we list ChaosVehicles
        // in SpModuleRules, then we must also list ChaosVehiclesPlugin in all of our uplugin files. To avoid this
        // unnecessary clutter, we only list ChaosVehicles in this Build.cs file, where it is actually needed. The
        // ChaosVehicles module is defined here:
        //     Engine/Plugins/Experimental/ChaosVehiclesPlugin/Source/ChaosVehicles
        //
        // Our Vehicle module also depends on the ChaosVehiclesCore module, which is defined here:
        //     Engine/Source/Runtime/Experimental/ChaosVehicles/ChaosVehiclesCore
        //
        // But since ChaosVehiclesCore is defined in Engine/Source, rather than Engine/Plugins, we can list it in
        // SpModuleRules without needing to add clutter to our uplugin files.

        PublicDependencyModuleNames.AddRange(new string[] {"ChaosVehicles", "SpComponents", "SpCore"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
