//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool;

public class SimulationController : CommonModuleRules
{
    public SimulationController(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        // As a matter of convenience, most Unreal module dependencies can be placed in CommonModuleRules, regardless
        // of whether or not the module is listed in our uplugin files. These Chaos dependencies are different. If we
        // place these Chaos module dependencies in CommonModuleRules, then we must also place ChaosVehiclesPlugin in
        // our uplugin files. So we only list these Chaos module dependencies in the Build.cs files where they are
        // needed. ChaosVehiclePlugin is treated differently by the Unreal Engine build system because it is a plugin
        // (i.e., it is stored in the Engine/Plugins directory, rather than the Engine/Source directory).
        PublicDependencyModuleNames.AddRange(new string[] {"ChaosVehicles", "ChaosVehiclesCore", "CoreUtils", "Vehicle"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
