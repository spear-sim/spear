//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
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
#include <HAL/Platform.h>                // uint8
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h>        // GENERATED_BODY, UCLASS, UENUM, UFUNCTION, UPROPERTY

#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"

#include "SpDebugManager.generated.h"

class UObject;

UENUM(BlueprintType)
enum class ESpDebugManagerEnum : uint8
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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Config, Category="SPEAR", DisplayName="My debug string")
    FString DebugString;

private:
    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Initialize(); // can't be const

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void Terminate(); // can't be const

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void LoadConfig(); // can't be const

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void SaveConfig(); // can't be const

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void PrintDebugString() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void GetAndSetObjectProperties(); // can't be const

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void CallFunctions(); // can't be const

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void CallSpFunc() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void CreateObjects() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void SubscribeToActorHitEvents() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void UnsubscribeFromActorHitEvents() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void ReadPixels() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void PrintLevelDebugInfo() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void PrintActorDebugInfo() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void PrintAllClassesDebugInfo() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void TestReinterpretAsVectorOf() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void TestInt64(); // can't be const

    UFUNCTION(BlueprintCallable, CallInEditor, Category="SPEAR")
    void TestFixedSizeArrays(); // can't be const

    // not intended to be BlueprintCallable

    UFUNCTION()
    FString GetString(FString Arg0, bool Arg1, int Arg2, FVector Arg3) const;

    UFUNCTION()
    int64 GetInt64(int64 Arg0) const; // echoes Arg0 so we can verify exact round-trips

    UFUNCTION()
    uint64 GetUInt64(uint64 Arg0) const; // echoes Arg0 so we can verify exact round-trips

    UFUNCTION()
    TArray<int64> GetArrayOfInt64(TArray<int64> Arg0) const; // echoes Arg0 to exercise the container arg + return path

    UFUNCTION()
    TArray<uint64> GetArrayOfUInt64(TArray<uint64> Arg0) const; // echoes Arg0 to exercise the container arg + return path

    UFUNCTION()
    FVector GetVector(FString Arg0, bool Arg1, int Arg2, FVector& Arg3) const;

    UFUNCTION()
    static UObject* GetWorldContextObject(const UObject* WorldContextObject, FString Arg0, bool Arg1);

    UFUNCTION()
    static void UpdateData(TMap<FString, FVector>& InMapFromStringToVector, TArray<FVector>& InArrayOfVectors);

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpFuncComponent* SpFuncComponent = nullptr;

    // not intended to be Blueprint-accessible

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
    TArray<ESpDebugManagerEnum> ArrayOfEnums;

    UPROPERTY()
    TMap<int, int> MapFromIntToInt;

    UPROPERTY()
    TMap<FString, FVector> MapFromStringToVector;

    UPROPERTY()
    TSet<FString> SetOfStrings;

    UPROPERTY()
    float FixedSizeArrayOfFloats[3];

    UPROPERTY()
    FString FixedSizeArrayOfStrings[2];

    UPROPERTY()
    FVector FixedSizeArrayOfVectors[2];

    UPROPERTY()
    int64 FixedSizeArrayOfInt64[2];

    std::unique_ptr<SharedMemoryRegion> shared_memory_region_;
    SpArraySharedMemoryView shared_memory_view_;
};
