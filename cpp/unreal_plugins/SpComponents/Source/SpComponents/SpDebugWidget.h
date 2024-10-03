//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Containers/Array.h>
#include <Containers/Map.h>
#include <Containers/Set.h>
#include <Containers/UnrealString.h> // FString
#include <GameFramework/Actor.h>
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/SharedMemoryRegion.h"

#include "SpComponents/SpFuncComponent.h"

#include "SpDebugWidget.generated.h"

class UObject;

UCLASS(ClassGroup="SPEAR", Config=Spear, HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ASpDebugWidget : public AActor
{
    GENERATED_BODY()
public: 
    ASpDebugWidget();
    ~ASpDebugWidget();

    // AActor interface
    void BeginDestroy() override;

private:
    UFUNCTION(CallInEditor, Category="SPEAR")
    void LoadConfig();
    UFUNCTION(CallInEditor, Category="SPEAR")
    void SaveConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void PrintDebugString();
    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="My debug string")
    FString DebugString;

    UFUNCTION(CallInEditor, Category="SPEAR")
    void GetAndSetObjectProperties();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void CallFunctions();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void CallSpFunc();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void CreateObjects();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void SubscribeToActorHitEvents();

    UFUNCTION()
    FString GetString(FString arg_0, bool arg_1, int arg_2, FVector arg_3);

    UFUNCTION()
    FVector GetVector(FString arg_0, bool arg_1, int arg_2, FVector& arg_3);

    UFUNCTION()
    static UObject* GetWorldContextObject(const UObject* world_context_object, FString arg_0, bool arg_1);

    UFUNCTION()
    static void UpdateData(TMap<FString, FVector>& map_from_string_to_vector, TArray<FVector>& array_of_vectors);

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpFuncComponent* SpFuncComponent = nullptr;

    UPROPERTY()
    FString MyString;

    UPROPERTY()
    TArray<int> ArrayOfInts;

    UPROPERTY()
    TArray<FVector> ArrayOfVectors;

    UPROPERTY()
    TArray<FString> ArrayOfStrings;

    UPROPERTY()
    TMap<int, int> MapFromIntToInt;

    UPROPERTY()
    TMap<FString, FVector> MapFromStringToVector;

    UPROPERTY()
    TMap<FString, FVector> OtherMapFromStringToVector;

    UPROPERTY()
    TSet<FString> SetOfStrings;

    void initializeSpFuncs();
    void terminateSpFuncs(); // don't call from destructor because SpFuncComponent might have been garbage-collected already

    std::unique_ptr<SharedMemoryRegion> shared_memory_region_ = nullptr;
    SpFuncSharedMemoryView shared_memory_view_;
};
