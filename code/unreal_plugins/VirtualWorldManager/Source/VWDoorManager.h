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

UCLASS()
class VIRTUALWORLDMANAGER_API UVWDoorManager : public UObject
{
    GENERATED_BODY()
public:
    UVWDoorManager();

    // load door settings
    bool loadData();
    // match setting door to actor in VW
    void matchDoorActor();
    // move each door respectively
    bool moveAllDoor(bool open);


private:
    bool isLoadSuccess = false;

    TArray<FDoorInfo> doorsData;

    UDataTable* doorsDataTable;
};
