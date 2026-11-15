//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Channels/MovieSceneChannelData.h>
#include <Channels/MovieSceneEvent.h>
#include <Containers/ArrayView.h>
#include <HAL/Platform.h>         // int32
#include <K2Node.h>
#include <K2Node_CustomEvent.h>
#include <Kismet/BlueprintFunctionLibrary.h>
#include <MovieSceneEventUtils.h>
#include <Sections/MovieSceneEventRepeaterSection.h>
#include <Sections/MovieSceneEventTriggerSection.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION

// private headers need custom include paths in SpUnrealTypesEditor.Build.cs
#include <KeysAndChannels/MovieSceneScriptingEvent.h> // UMovieSceneScriptingEventKey

#include "SpCore/Assert.h"

#include "SpMovieSceneEventUtils.generated.h"

class UBlueprint;

UCLASS()
class USpMovieSceneEventUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static UK2Node* BindNewUserFacingEventForKey(UMovieSceneScriptingEventKey* Key, UMovieSceneEventTriggerSection* EventTriggerSection, UBlueprint* Blueprint)
    {
        SP_ASSERT(EventTriggerSection);
        SP_ASSERT(Key);

        TMovieSceneChannelData<FMovieSceneEvent> channel_data = EventTriggerSection->EventChannel.GetData();
        TArrayView<FMovieSceneEvent> values = channel_data.GetValues();
        const int32 index = channel_data.GetIndex(Key->KeyHandle);
        SP_ASSERT(index >= 0 && index < values.Num());
        FMovieSceneEvent* event = &(values[index]);

        return FMovieSceneEventUtils::BindNewUserFacingEvent(event, EventTriggerSection, Blueprint);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static UK2Node* BindNewUserFacingEventForRepeaterSection(UMovieSceneEventRepeaterSection* EventRepeaterSection, UBlueprint* Blueprint)
    {
        SP_ASSERT(EventRepeaterSection);

        FMovieSceneEvent* event = &(EventRepeaterSection->Event);

        return FMovieSceneEventUtils::BindNewUserFacingEvent(event, EventRepeaterSection, Blueprint);
    }
};
