//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSimEditor/SpearSimEditorUtils.h"

#include <Containers/UnrealString.h>
#include <CoreMinimal.h>
#include <GameFramework/Actor.h>

#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

class FObjectInitializer;

ASpearSimEditorUtils::ASpearSimEditorUtils(const FObjectInitializer& object_initializer)
{
    SP_LOG_CURRENT_FUNCTION();
}

ASpearSimEditorUtils::~ASpearSimEditorUtils()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpearSimEditorUtils::loadConfig()
{
    LoadConfig();
}

void ASpearSimEditorUtils::saveConfig()
{
    SaveConfig();
}

void ASpearSimEditorUtils::printDummyString()
{
    SP_LOG("dummy_string_: ", Unreal::toStdString(dummy_string_));
}
