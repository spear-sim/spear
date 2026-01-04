//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using System.IO;       // Directory, FileSystemInfo, Path
using UnrealBuildTool; // ReadOnlyTargetRules

public class SpUnrealTypesEditor : SpModuleRules
{
    public SpUnrealTypesEditor(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        PublicDependencyModuleNames.AddRange(new string[] {"MovieScene", "MovieSceneTools", "MovieSceneTracks", "Sequencer", "SequencerScripting", "SpCore", "UnrealEd"});
        PrivateDependencyModuleNames.AddRange(new string[] {});

        // Needed to expose a private header in SequencerScripting
        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(target.RelativeEnginePath, "Plugins", "MovieScene", "SequencerScripting", "Source", "SequencerScripting", "Private")));
    }
}
