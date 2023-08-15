//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSimEditor/DebugWidget.h"

#include <CoreMinimal.h> // FString
#include <GameFramework/Actor.h>

#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

class FObjectInitializer;

ADebugWidget::ADebugWidget(const FObjectInitializer& object_initializer)
{
    SP_LOG_CURRENT_FUNCTION();
}

ADebugWidget::~ADebugWidget()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ADebugWidget::loadConfig()
{
    LoadConfig();
}

void ADebugWidget::saveConfig()
{
    SaveConfig();
}

void ADebugWidget::printDebugString()
{
    SP_LOG("debug_string_: ", Unreal::toStdString(debug_string_));
}
