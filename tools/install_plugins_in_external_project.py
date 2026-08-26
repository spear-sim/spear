#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import json
import numpy as np
import os
import shutil
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--external-project-dir", required=True)
args = parser.parse_args()


if __name__ == "__main__":

    external_project_dir = os.path.realpath(args.external_project_dir)
    uprojects = glob.glob(os.path.realpath(os.path.join(external_project_dir, "*.uproject")))
    assert len(uprojects) == 1
    uproject = uprojects[0]
    uproject_dir = os.path.dirname(uproject)

    #
    # update uproject
    #

    spear.log("Reading uproject file: ", uproject)

    uproject_dict = {}
    with open(uproject) as f:
        uproject_dict = json.load(f)

    save = False
    target_plugins_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_plugins")).replace("\\\\", "/").replace("\\", "/")

    if "AdditionalPluginDirectories" in uproject_dict:
        found = False
        for additional_plugins_dir in uproject_dict["AdditionalPluginDirectories"]:
            candidate_plugins_dir = additional_plugins_dir.replace("/", os.path.sep)
            if os.path.isabs(candidate_plugins_dir):
                candidate_plugins_dir = candidate_plugins_dir.replace("\\\\", "/").replace("\\", "/")
            else:
                candidate_plugins_dir = os.path.realpath(os.path.join(uproject_dir, candidate_plugins_dir)).replace("\\\\", "/").replace("\\", "/")
            if candidate_plugins_dir == target_plugins_dir:
                spear.log("AdditionalPluginDirectories already has a matching entry: ", additional_plugins_dir)
                found = True
                break
        if not found:
            spear.log("Adding to AdditionalPluginDirectories entry: ", target_plugins_dir)
            uproject_dict["AdditionalPluginDirectories"].append(target_plugins_dir)
            save = True
    else:
        spear.log("Creating AdditionalPluginDirectories entry and adding: ", target_plugins_dir)
        uproject_dict["AdditionalPluginDirectories"] = [target_plugins_dir]
        save = True

    if save:
        spear.log("Saving uproject: ", uproject)
        with open(uproject, "w") as f:
            json.dump(uproject_dict, f, indent=4, sort_keys=False)
            f.write("\n")

    #
    # create a minimal Source directory if the project doesn't already have one, so the SPEAR plugins have a
    # Target.cs to build against, and so a monolithic build has a primary game module to link against (e.g.,
    # for content-only projects that don't have any C++ code of their own)
    #

    source_dir = os.path.realpath(os.path.join(external_project_dir, "Source"))

    if os.path.exists(source_dir):
        spear.log("Project already has a Source directory, so we assume a Target.cs and a primary game module already exist: ", source_dir)
    else:
        spear.log("Project has no Source directory, creating a minimal one so the SPEAR plugins can be built: ", source_dir)

        project_name = os.path.splitext(os.path.basename(uproject))[0]
        module_dir = os.path.realpath(os.path.join(source_dir, project_name))

        sp_target_rules_src_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim", "Source", "SpTargetRules.Target.cs"))
        sp_target_rules_dst_file = os.path.realpath(os.path.join(source_dir, "SpTargetRules.Target.cs"))
        game_target_file = os.path.realpath(os.path.join(source_dir, f"{project_name}.Target.cs"))
        editor_target_file = os.path.realpath(os.path.join(source_dir, f"{project_name}Editor.Target.cs"))
        module_build_file = os.path.realpath(os.path.join(module_dir, f"{project_name}.Build.cs"))
        module_header_file = os.path.realpath(os.path.join(module_dir, f"{project_name}.h"))
        module_source_file = os.path.realpath(os.path.join(module_dir, f"{project_name}.cpp"))

        os.makedirs(module_dir, exist_ok=True)

        spear.log("Copying: ", sp_target_rules_src_file, " -> ", sp_target_rules_dst_file)
        shutil.copy(sp_target_rules_src_file, sp_target_rules_dst_file)

        # We need a primary game module (rather than just Target.cs files) because a monolithic Game/Client
        # build links all its modules into a single executable, and the boilerplate that provides symbols such
        # as GInternalProjectName and GIsGameAgnosticExe is only emitted by whichever module calls
        # IMPLEMENT_PRIMARY_GAME_MODULE(...). None of the SPEAR plugin modules call this macro, so a
        # content-only project with no primary game module of its own fails to link as a standalone Game/Client
        # target, even though its Editor target (which only builds plugin modules as dylibs that get loaded
        # into the already-built UnrealEditor binary) builds fine.

        game_target_str = \
f"""//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

// Auto-generated by spear/tools/install_plugins_in_external_project.py

using UnrealBuildTool; // TargetInfo

public class {project_name}Target : SpTargetRulesTarget
{{
    public {project_name}Target(TargetInfo targetInfo) : base(targetInfo)
    {{
        SP_LOG_CURRENT_FUNCTION();

        Type = TargetType.Game;
        ExtraModuleNames.Add("{project_name}");
    }}
}}
"""

        editor_target_str = \
f"""//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

// Auto-generated by spear/tools/install_plugins_in_external_project.py

using UnrealBuildTool; // TargetInfo

public class {project_name}EditorTarget : SpTargetRulesTarget
{{
    public {project_name}EditorTarget(TargetInfo targetInfo) : base(targetInfo)
    {{
        SP_LOG_CURRENT_FUNCTION();

        Type = TargetType.Editor;
        ExtraModuleNames.Add("{project_name}");
    }}
}}
"""

        module_build_str = \
f"""//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

// Auto-generated by spear/tools/install_plugins_in_external_project.py

using UnrealBuildTool; // ReadOnlyTargetRules

public class {project_name} : SpModuleRules
{{
    // Deliberately not passing bLinkThirdPartyLibraries: true here (see SpModuleRules.Build.cs), because this
    // module lives inside the external project's own directory tree, not inside the SPEAR repo, so
    // SpModuleRules can't compute a valid path to spear/third_party. This module gets everything it needs
    // transitively through its PublicDependencyModuleNames dependency on SpCore.
    public {project_name}(ReadOnlyTargetRules target) : base(target)
    {{
        SP_LOG_CURRENT_FUNCTION();

        PublicDependencyModuleNames.AddRange(new string[] {{"SpCore"}});
        PrivateDependencyModuleNames.AddRange(new string[] {{}});
    }}
}}
"""

        module_header_str = \
f"""//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

// Auto-generated by spear/tools/install_plugins_in_external_project.py

#pragma once

#include <Modules/ModuleInterface.h>

class {project_name} : public IModuleInterface
{{
public:
    void StartupModule() override;
    void ShutdownModule() override;
}};
"""

        module_source_str = \
f"""//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

// Auto-generated by spear/tools/install_plugins_in_external_project.py

#include "{project_name}/{project_name}.h"

#include <Modules/ModuleManager.h> // IMPLEMENT_PRIMARY_GAME_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"

void {project_name}::StartupModule()
{{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();
}}

void {project_name}::ShutdownModule()
{{
    SP_LOG_CURRENT_FUNCTION();
}}

// must be used at least once per game, even if the module doesn't implement any Unreal classes
IMPLEMENT_PRIMARY_GAME_MODULE({project_name}, {project_name}, "{project_name}"); // ModuleImplClass, ModuleName, DEPRECATED_GameName
"""

        spear.log("Creating: ", game_target_file)
        with open(game_target_file, "w") as f:
            f.write(game_target_str)

        spear.log("Creating: ", editor_target_file)
        with open(editor_target_file, "w") as f:
            f.write(editor_target_str)

        spear.log("Creating: ", module_build_file)
        with open(module_build_file, "w") as f:
            f.write(module_build_str)

        spear.log("Creating: ", module_header_file)
        with open(module_header_file, "w") as f:
            f.write(module_header_str)

        spear.log("Creating: ", module_source_file)
        with open(module_source_file, "w") as f:
            f.write(module_source_str)

        # register the new primary game module in the uproject so the engine knows to load it
        spear.log("Adding Modules entry to uproject and saving: ", uproject)
        uproject_dict["Modules"] = uproject_dict.get("Modules", []) + [
            {"Name": project_name, "Type": "Runtime", "LoadingPhase": "Default", "AdditionalDependencies": ["SpCore"]}]
        with open(uproject, "w") as f:
            json.dump(uproject_dict, f, indent=4, sort_keys=False)
            f.write("\n")

    #
    # update DefaultGame.ini
    #

    config_dir = os.path.realpath(os.path.join(external_project_dir, "Config"))
    default_game_ini_file = os.path.realpath(os.path.join(config_dir, "DefaultGame.ini"))
    default_game_bak_file = os.path.realpath(os.path.join(config_dir, "DefaultGame.ini.bak"))
    backup = False
    save = False
    sections = []

    if os.path.exists(default_game_ini_file):

        spear.log("Reading INI file: ", default_game_ini_file)
        with open(default_game_ini_file, "r") as f:
            lines = [ line for line in f ]

        current_section = {"heading": None, "lines": []}
        for line in lines:
            line_stripped = line.partition(";")[0].strip()
            if line_stripped.startswith("[") and line_stripped.endswith("]"):
                sections.append(current_section)
                current_section = {"heading": line_stripped.replace("[", "").replace("]", ""), "lines": [line]}
            else:
                current_section["lines"].append(line)
        sections.append(current_section)

        section_indices = np.nonzero([ section["heading"] == "/Script/UnrealEd.ProjectPackagingSettings" for section in sections ])[0]

        if len(section_indices) > 0:
            spear.log("Found /Script/UnrealEd.ProjectPackagingSettings section in existing INI file...")

            found_entry = False
            for section_index in section_indices:
                for line in sections[section_index]["lines"]:
                    line_stripped = line.partition(";")[0].strip()
                    if line_stripped == '+DirectoriesToAlwaysCook=(Path="/SpContent")':
                        spear.log('Found +DirectoriesToAlwaysCook=(Path="/SpContent") entry in existing section...')
                        found_entry = True
                        break
                if found_entry:
                    break

            if not found_entry:
                backup = True
                save = True
                section_index = section_indices[-1]
                section = sections[section_index]
                nonempty_line_indices = np.nonzero([ line.strip() != "" for line in section["lines"] ])[0]

                if len(nonempty_line_indices) > 0:
                    spear.log('Inserting +DirectoriesToAlwaysCook=(Path="/SpContent") entry in existing section...')
                    section["lines"].insert(nonempty_line_indices[-1] + 1, '+DirectoriesToAlwaysCook=(Path="/SpContent") ; auto-generated by spear/tools/install_plugins_in_external_project.py\n')
                else:
                    spear.log('Appending +DirectoriesToAlwaysCook=(Path="/SpContent") entry to existing section...')
                    section["lines"].append('+DirectoriesToAlwaysCook=(Path="/SpContent") ; auto-generated by spear/tools/install_plugins_in_external_project.py\n')

        else:
            backup = True
            save = True
            spear.log("Adding /Script/UnrealEd.ProjectPackagingSettings to INI file...")

            if len(sections) > 0 and len(sections[-1]["lines"]) > 0 and sections[-1]["lines"][-1].strip() == "":
                lines_prefix = []
            else:
                lines_prefix = ["\n"]

            section = {
                "heading": "/Script/UnrealEd.ProjectPackagingSettings",
                "lines": lines_prefix + ["[/Script/UnrealEd.ProjectPackagingSettings]\n", '+DirectoriesToAlwaysCook=(Path="/SpContent") ; auto-generated by spear/tools/install_plugins_in_external_project.py\n']}
            sections.append(section)

    else:
        backup = False
        save = True
        spear.log("Creating new INI file: ", default_game_ini_file)

        section = {
            "heading": "/Script/UnrealEd.ProjectPackagingSettings",
            "lines": ["[/Script/UnrealEd.ProjectPackagingSettings]\n", '+DirectoriesToAlwaysCook=(Path="/SpContent") ; auto-generated by spear/tools/install_plugins_in_external_project.py\n']}
        sections.append(section)

    if backup:
        spear.log("Creating backup INI file: ", default_game_bak_file)
        shutil.copy(default_game_ini_file, default_game_bak_file)

    if save:
        spear.log("Saving INI file...")
        os.makedirs(config_dir, exist_ok=True)
        with open(default_game_ini_file, "w") as f:
            for section in sections:
                for line in section["lines"]:
                    f.write(line)

    spear.log("Done.")
