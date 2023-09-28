//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "CoreUtils/SpearComponent.h"

#include "Components/ActorComponent.h"
#include "Delegates/IDelegateInstance.h"	//FDelegateHandle
#include "GameFramework/Actor.h"
#include "Misc/CoreDelegates.h"				//FCoreDelegates
#include "UObject/UnrealType.h"				//FPropertyChangedEvent

#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

USpearComponent::USpearComponent(const FObjectInitializer& object_initializer) : UActorComponent(object_initializer)
{
	SP_LOG_CURRENT_FUNCTION();

#if WITH_EDITOR
	on_actor_label_changed_handle_ = FCoreDelegates::OnActorLabelChanged.AddUObject(this, &USpearComponent::onActorLabelChangedEventHandler);
#endif
}

USpearComponent::~USpearComponent()
{
	SP_LOG_CURRENT_FUNCTION();

#if WITH_EDITOR
	FCoreDelegates::OnActorLabelChanged.Remove(on_actor_label_changed_handle_);
	on_actor_label_changed_handle_.Reset();
#endif
}

#if WITH_EDITOR
void USpearComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	SP_LOG_CURRENT_FUNCTION();

	if (PropertyChangedEvent.GetMemberPropertyName() == GET_MEMBER_NAME_CHECKED(USpearComponent, parent_actor_label_name_)) {
		AActor* owner = this->GetOwner();
		owner->SetActorLabel(parent_actor_label_name_);
	}
	UActorComponent::PostEditChangeProperty(PropertyChangedEvent);
}

void USpearComponent::onActorLabelChangedEventHandler(AActor* actor)
{
	if (actor == this->GetOwner()) {
		parent_actor_label_name_ = actor->GetActorLabel();
	}
}
#endif
