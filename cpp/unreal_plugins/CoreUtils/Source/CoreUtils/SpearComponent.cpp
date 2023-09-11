//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "CoreUtils/SpearComponent.h"

#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

USpearComponent::USpearComponent(const FObjectInitializer& object_initializer) : UActorComponent(object_initializer)
{
	SP_LOG_CURRENT_FUNCTION();
}

USpearComponent::~USpearComponent()
{
	SP_LOG_CURRENT_FUNCTION();
}

#if WITH_EDITOR
void USpearComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	SP_LOG_CURRENT_FUNCTION();

	SP_LOG("Property being edited is ", Unreal::toStdString(PropertyChangedEvent.GetMemberPropertyName()));
	if (PropertyChangedEvent.GetMemberPropertyName() == GET_MEMBER_NAME_CHECKED(USpearComponent, parent_actor_label_name_)) {
		AActor* owner = this->GetOwner();
		owner->SetActorLabel(parent_actor_label_name_);
	}
	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif //WITH_EDITOR
