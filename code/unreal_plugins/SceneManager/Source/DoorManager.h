#pragma once

#include <CoreMinimal.h>
#include <Engine/DataTable.h>

#include "DoorManager.generated.h"

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

class SCENEMANAGER_API DoorManager
{
public:
    // initialize door info for current level. only valid for InteriorSim scene
    static bool initLevelDoorInfo(UWorld* world);
    // move each door respectively
    static bool moveAllDoor(bool open);

private:
    // match door info with actor in current level by position
    static void matchDoorActor(UWorld* world);
    // storing all door infos and their door actor
    static TArray<FDoorInfo> level_door_info_;
};
