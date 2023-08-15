//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "SpearSimEditor/SpearSimEditorUnrealEdEngine.h"

#include <map>
#include <string>
#include <vector>


#include <Components/StaticMeshComponent.h>
#include <Editor/UnrealEdEngine.h>
#include <GameFramework/Actor.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"

USpearSimEditorUnrealEdEngine::USpearSimEditorUnrealEdEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

USpearSimEditorUnrealEdEngine::~USpearSimEditorUnrealEdEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

bool USpearSimEditorUnrealEdEngine::Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device)
{
    std::string cmd_str = Unreal::toStdString(cmd);
    SP_LOG(cmd_str);

    std::vector<std::string> cmd_list = Std::tokenize(cmd_str, " ");

    std::map<std::string, AActor*> all_actors_name_ref_map = Unreal::findActorsByTagAllAsMap(world, {});
    std::map<AActor*, UStaticMeshComponent*> mesh_actor_ref_map;

    for (auto& element: all_actors_name_ref_map) {
        
    }

    if (cmd_list.at(0) == "setObjectLocation") {
        SP_ASSERT(cmd_list.size() == 5);
        SP_ASSERT(all_actors_name_ref_map.count(cmd_list.at(1)));

        
    }


    return UUnrealEdEngine::Exec(world, cmd, output_device);
}
