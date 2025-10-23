//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Containers/Array.h>
#include <Containers/Map.h>
#include <Containers/Set.h>
#include <Containers/UnrealString.h>     // FString
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/EngineTypes.h>          // EEndPlayReason
#include <GameFramework/Actor.h>
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h>        // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"

#include "SpDebugManager.generated.h"

class UObject;

UENUM()
enum class EDebugManagerEnum
{
    Hello,
    World
};

UCLASS(ClassGroup="SPEAR", Config=Spear, HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpDebugManager : public AActor
{
    GENERATED_BODY()
public: 
    ASpDebugManager();
    ~ASpDebugManager() override;

    // AActor interface
    void PostInitProperties() override;
    void PostActorCreated() override;
    void PostLoad() override;
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;
    void BeginDestroy() override;

private:
    UFUNCTION(CallInEditor, Category="SPEAR")
    void LoadConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void SaveConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void PrintDebugString() const;

    UFUNCTION(CallInEditor, Category="SPEAR")
    void GetAndSetObjectProperties();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void CallFunctions();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void CallSpFunc() const;

    UFUNCTION(CallInEditor, Category="SPEAR")
    void CreateObjects();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void SubscribeToActorHitEvents();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void ReadPixels();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void PrintDebugInfo();

    UFUNCTION()
    FString GetString(FString Arg0, bool Arg1, int Arg2, FVector Arg3) const;

    UFUNCTION()
    FVector GetVector(FString Arg0, bool Arg1, int Arg2, FVector& Arg3) const;

    UFUNCTION()
    static UObject* GetWorldContextObject(const UObject* WorldContextObject, FString Arg0, bool Arg1);

    UFUNCTION()
    static void UpdateData(TMap<FString, FVector>& InMapFromStringToVector, TArray<FVector>& InArrayOfVectors);

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpFuncComponent* SpFuncComponent = nullptr;

    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="My debug string")
    FString DebugString;

    UPROPERTY()
    FString MyString;

    UPROPERTY()
    TArray<int> ArrayOfInts;

    UPROPERTY()
    TArray<FVector> ArrayOfVectors;

    UPROPERTY()
    TArray<FString> ArrayOfStrings;

    UPROPERTY()
    TArray<UObject*> ArrayOfPointers;

    UPROPERTY()
    TArray<EDebugManagerEnum> ArrayOfEnums;

    UPROPERTY()
    TMap<int, int> MapFromIntToInt;

    UPROPERTY()
    TMap<FString, FVector> MapFromStringToVector;

    UPROPERTY()
    TMap<FString, FVector> OtherMapFromStringToVector;

    UPROPERTY()
    TSet<FString> SetOfStrings;

    void initializeSpFunc();
    void terminateSpFunc();

    bool is_game_world_ = false;

    std::unique_ptr<SharedMemoryRegion> shared_memory_region_ = nullptr;
    SpArraySharedMemoryView shared_memory_view_;
};
