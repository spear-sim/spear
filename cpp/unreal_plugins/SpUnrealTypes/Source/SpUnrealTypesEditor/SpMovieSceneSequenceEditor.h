//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <MovieSceneSequence.h>
#include <MovieSceneSequenceEditor.h>

#include "SpCore/Assert.h"

#include "SpMovieSceneSequenceEditor.generated.h"

class UBlueprint;

UCLASS()
class USpMovieSceneSequenceEditor : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")    
    static UBlueprint* GetOrCreateDirectorBlueprint(UMovieSceneSequence* Sequence)
    {
        SP_ASSERT(Sequence);
        FMovieSceneSequenceEditor* sequence_editor = FMovieSceneSequenceEditor::Find(Sequence);
        SP_ASSERT(sequence_editor);
        return sequence_editor->GetOrCreateDirectorBlueprint(Sequence);
    }
};
