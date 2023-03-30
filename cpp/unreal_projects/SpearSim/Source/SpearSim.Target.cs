//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;
using System.IO;
using UnrealBuildTool;

public class SpearSimTarget : TargetRules
{
    public SpearSimTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange(new string[] { "SpearSim" });

        if (Target.Platform == UnrealTargetPlatform.Win64) {

            // On Windows, we need to build an additional app so that calls to UE_Log and writes to std::cout are visible in the terminal.
            bBuildAdditionalConsoleApp = true;

        } else if (Target.Platform == UnrealTargetPlatform.Mac || Target.Platform == UnrealTargetPlatform.Linux) {

            // On macOS and Linux, we need to remap the paths of our symbolic links as we're compiling our executable, so the paths that get
            // written into the application's debug symbols aren't symbolic links. This is necessary to enable debugging in XCode and LLDB.
            bOverrideBuildEnvironment = true;
            AdditionalCompilerArguments = "";

            AdditionalCompilerArguments +=
                " -ffile-prefix-map=" +
                Path.Combine(ProjectFile.Directory.FullName, "Plugins") + "=" +
                Path.GetFullPath(Path.Combine(ProjectFile.Directory.FullName, "..", "..", "unreal_plugins"));

            AdditionalCompilerArguments +=
                " -ffile-prefix-map=" +
                Path.Combine(ProjectFile.Directory.FullName, "ThirdParty") + "=" +
                Path.GetFullPath(Path.Combine(ProjectFile.Directory.FullName, "..", "..", "..", "third_party"));

            foreach (string plugin in Directory.GetDirectories(Path.Combine(ProjectFile.Directory.FullName, "Plugins"))) {
                AdditionalCompilerArguments +=
                    " -ffile-prefix-map=" +
                    Path.Combine(plugin, "ThirdParty") + "=" +
                    Path.GetFullPath(Path.Combine(ProjectFile.Directory.FullName, "..", "..", "..", "third_party"));
            }
        }

        // We can't throw an exception here, because when we invoke GenerateProjectFiles.bat or GenerateProjectFiles.sh (packaged with
        // the Unreal Engine), instances of this class gets created with Target.Platform set to many different platforms (e.g., iOS, tvOS,
        // Win32, etc).
        // } else {
        //    throw new Exception("[SPEAR | SpearSim.Target.cs] Target.Platform == " + Target.Platform);            
        // }
    }
}
