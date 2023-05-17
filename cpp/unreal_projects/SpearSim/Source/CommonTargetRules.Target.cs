//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;
using System.IO;
using System.Runtime.CompilerServices;
using UnrealBuildTool;

public class CommonTargetRulesTarget : TargetRules
{
    public CommonTargetRulesTarget(TargetInfo target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        // We need to set this to something other than Game or Editor or Program in order to successfully generate Visual Studio project files.
        // Needs to be overridden in derived classes.
        Type = TargetType.Client;

        // Added to projects by default in UE 5.2. Note that the default value in UE 5.2 preview 2 for IncludeOrderVersion is
        // EngineIncludeOrderVersion.Unreal5_1, but that triggers a build warning.
        DefaultBuildSettings = BuildSettingsVersion.V2;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_2;

        if (target.Platform == UnrealTargetPlatform.Win64) {

            // On Windows, we need to build an additional app so that calls to UE_Log and writes to std::cout are visible in the terminal.
            bBuildAdditionalConsoleApp = true;

            // Sometimes useful for debugging
            // bOverrideBuildEnvironment = true;
            // AdditionalCompilerArguments = "/showIncludes";

        } else if (target.Platform == UnrealTargetPlatform.Mac || target.Platform == UnrealTargetPlatform.Linux) {

            // On macOS and Linux, we need to remap the paths of our symbolic links as we're compiling our executable, so the paths that get
            // written into the application's debug symbols aren't symbolic links. This is necessary to enable debugging in XCode and LLDB.
            bOverrideBuildEnvironment = true;
            AdditionalCompilerArguments = "";

            string arg = "";
            SP_LOG("Additional compiler arguments:");

            foreach (string pluginDir in Directory.GetDirectories(Path.Combine(ProjectFile.Directory.FullName, "Plugins"))) {
                string plugin = (new DirectoryInfo(pluginDir)).Name;

                // Do the most specific substitution first. If we do a less specific substitution first, then we might not ever perform the
                // more specific substitution, depending on how the -ffile-prefix-map argument is handled by the compiler.                

                // Old: path/to/spear/cpp/unreal_projects/SpearSim/Plugins/MyPlugin/ThirdParty
                // New: path/to/spear/third_party
                arg =
                    " -ffile-prefix-map=" +
                    Path.GetFullPath(Path.Combine(pluginDir, "ThirdParty")) + "=" +
                    Path.GetFullPath(Path.Combine(ProjectFile.Directory.FullName, "..", "..", "..", "third_party"));
                AdditionalCompilerArguments += arg;
                SP_LOG("    " + arg);

                // Old: path/to/spear/cpp/unreal_projects/SpearSim/Plugins/MyPlugin
                // New: path/to/spear/cpp/unreal_plugins/MyPlugin
                arg =
                    " -ffile-prefix-map=" +
                    Path.GetFullPath(Path.Combine(pluginDir)) + "=" +
                    Path.GetFullPath(Path.Combine(ProjectFile.Directory.FullName, "..", "..", "unreal_plugins", plugin));
                AdditionalCompilerArguments += arg;
                SP_LOG("    " + arg);
            }

            // Old: path/to/spear/cpp/unreal_projects/SpearSim/ThirdParty
            // New: path/to/spear/third_party
            arg =
                " -ffile-prefix-map=" +
                Path.GetFullPath(Path.Combine(ProjectFile.Directory.FullName, "ThirdParty")) + "=" +
                Path.GetFullPath(Path.Combine(ProjectFile.Directory.FullName, "..", "..", "..", "third_party"));
            AdditionalCompilerArguments += arg;
            SP_LOG("    " + arg);

        } else {
            throw new Exception(SP_LOG_GET_PREFIX() + "Unexpected target platform: " + target.Platform);
        }
    }

    protected void SP_LOG(string message, [CallerFilePath] string filePath="")
    {
        Console.WriteLine(GetPrefix(filePath) + message);
    }

    protected void SP_LOG_CURRENT_FUNCTION([CallerFilePath] string filePath="", [CallerMemberName] string memberName="")
    {
        Console.WriteLine(GetPrefix(filePath) + GetCurrentFunctionExpanded(memberName));
    }

    protected string SP_LOG_GET_PREFIX([CallerFilePath] string filePath="")
    {
        return GetPrefix(filePath);
    }

    private string GetPrefix(string filePath)
    {
        return "[SPEAR | " + GetCurrentFileAbbreviated(filePath) + "] ";
    }

    private string GetCurrentFileAbbreviated(string filePath)
    {
        return Path.GetFileName(filePath);
    }

    private string GetCurrentFunctionExpanded(string memberName)
    {
        return this.GetType() + "." + ((memberName == ".ctor") ? this.GetType() : memberName);
    }
}
