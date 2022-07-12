#pragma once

#include "CoreMinimal.h"

#include "Engine/DataTable.h"
#include "VWDoorManager.generated.h"

USTRUCT(BlueprintType)
struct FDoorInfo
{
    GENERATED_USTRUCT_BODY()
    //door mode: clockwise | counter-clockwise | sliding
    UPROPERTY()
    FString mode;
    // whether one side of the door is outside the house
    UPROPERTY()
    bool isInnerDoor;
    //world position
    UPROPERTY()
    FVector position;
    // orientation
    UPROPERTY()
    FVector normal;

    AActor* doorActor = nullptr;
};

USTRUCT(BlueprintType)
struct FSceneDoorInfo : public FTableRowBase
{
    GENERATED_USTRUCT_BODY()
    UPROPERTY()
    FString virtualWorldId;
    UPROPERTY()
    TArray<FDoorInfo> doors;
};

class VIRTUALWORLDMANAGER_API UVWDoorManager
{
public:
    // load door settings
    static bool initLevelDoorInfo(UWorld* world);
    // move each door respectively
    static bool moveAllDoor(bool open);

private:
    // match setting door to actor in current level
    static void matchDoorActor(UWorld* world);
    // storing all door info and door actor
    static TArray<FDoorInfo> level_door_info_;
};
