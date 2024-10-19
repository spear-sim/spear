//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System;                          // Console, Exception
using System.IO;                       // Directory, DirectoryInfo, Path
using System.Runtime.CompilerServices; // CallerFilePath, CallerLineNumber, CallerMemberName
using UnrealBuildTool;                 // TargetInfo, TargetRules

public class SpTargetRulesTarget : TargetRules
{
    public SpTargetRulesTarget(TargetInfo targetInfo) : base(targetInfo)
    {
        SP_LOG_CURRENT_FUNCTION();

        // We need to set this to something other than Game or Editor or Program in order to successfully generate Visual Studio project files.
        // Needs to be overridden in derived classes.
        Type = TargetType.Client;

        // Added to projects by default in UE 5.4.
        DefaultBuildSettings = BuildSettingsVersion.V5;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_4;
        ExtraModuleNames.Add("SpearSim");

        if (targetInfo.Platform == UnrealTargetPlatform.Win64) {

            // On Windows, we need to build an additional app so that calls to UE_LOG and writes to std::cout are visible in the terminal.
            bBuildAdditionalConsoleApp = true;

            // Sometimes useful for debugging
            // bOverrideBuildEnvironment = true;
            // AdditionalCompilerArguments = "/showIncludes";

        } else if (targetInfo.Platform == UnrealTargetPlatform.Mac || targetInfo.Platform == UnrealTargetPlatform.Linux) {

            // pass

        } else if (targetInfo.Platform == UnrealTargetPlatform.IOS || targetInfo.Platform == UnrealTargetPlatform.TVOS) {
            SP_LOG("NOTE: We only expect to see targetInfo.Platform == UnrealTargetPlatform.IOS or targetInfo.Platform == UnrealTargetPlatform.TVOS when we're on macOS and we're attempting to generate XCode project files. If we're not on macOS generating XCode project files, target.Platform == UnrealTargetPlatform.IOS and target.Platform == UnrealTargetPlatform.TVOS are unexpected.");

        } else if (targetInfo.Platform == UnrealTargetPlatform.LinuxArm64) {
            SP_LOG("NOTE: We only expect to see targetInfo.Platform == UnrealTargetPlatform.LinuxArm64 when we're on Linux and the editor is attempting to open a uproject for the first time. If the editor is not attempting to open a uproject for the first time on Linux, target.Platform == UnrealTargetPlatform.LinuxArm64 is unexpected.");

        } else {
            throw new Exception(SP_LOG_GET_PREFIX() + "Unexpected target platform: " + targetInfo.Platform);
        }
    }

    protected void SP_LOG(string message, [CallerFilePath] string filePath="", [CallerLineNumber] int lineNumber=0)
    {
        Console.WriteLine(GetPrefix(filePath, lineNumber) + message);
    }

    protected void SP_LOG_CURRENT_FUNCTION([CallerFilePath] string filePath="", [CallerLineNumber] int lineNumber=0, [CallerMemberName] string memberName="")
    {
        Console.WriteLine(GetPrefix(filePath, lineNumber) + GetCurrentFunctionExpanded(memberName));
    }

    protected string SP_LOG_GET_PREFIX([CallerFilePath] string filePath="", [CallerLineNumber] int lineNumber=0)
    {
        return GetPrefix(filePath, lineNumber);
    }

    private string GetPrefix(string filePath, int lineNumber)
    {
        return "[SPEAR | " + GetCurrentFileAbbreviated(filePath) + ":" + lineNumber.ToString("D4") + "] ";
    }

    private string GetCurrentFileAbbreviated(string filePath)
    {
        return Path.GetFileName(filePath);
    }

    private string GetCurrentFunctionExpanded(string memberName)
    {
        string sep = memberName.StartsWith(".") ? "" : ".";
        return this.GetType() + sep + memberName;
    }
}
