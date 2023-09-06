//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "CoreUtils/SpearComponent.h"

#include "CoreUtils/Log.h"

USpearComponent::USpearComponent(const FObjectInitializer& object_initializer) : UActorComponent(object_initializer)
{
	SP_LOG_CURRENT_FUNCTION();
}

USpearComponent::~USpearComponent()
{
	SP_LOG_CURRENT_FUNCTION();
}
