//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/Array.h>
#include <Containers/Map.h>
#include <Containers/UnrealString.h> // FString
#include <GameFramework/Actor.h>
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "DebugWidget.generated.h"

class UObject;

UCLASS(Config=Spear, HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ADebugWidget : public AActor
{
    GENERATED_BODY()
public: 
    ADebugWidget();
    ~ADebugWidget();

private:
    UFUNCTION(CallInEditor, Category="SPEAR")
    void LoadConfig();
    UFUNCTION(CallInEditor, Category="SPEAR")
    void SaveConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void PrintDebugString();
    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="Debug string")
    FString DebugString;

    UFUNCTION(CallInEditor, Category="SPEAR")
    void GetAndSetObjectProperties();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void CallFunctions();

    UFUNCTION(CallInEditor, Category = "SPEAR")
    void SubscribeToActorHitEvents();

    UFUNCTION()
    FString GetString(FString arg_0, bool arg_1, int arg_2, FVector arg_3);

    UFUNCTION()
    FVector GetVector(FString arg_0, bool arg_1, int arg_2, FVector& arg_3);

    UFUNCTION()
    static UObject* GetWorldContextObject(const UObject* world_context_object, FString arg_0, bool arg_1);

    UPROPERTY()
    TArray<int> ArrayOfInts;

    UPROPERTY()
    TArray<FVector> ArrayOfVectors;

    TMap<FString, FVector> MapFromStringToVector;
};
