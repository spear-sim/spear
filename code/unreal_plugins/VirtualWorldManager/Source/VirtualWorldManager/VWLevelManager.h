#pragma once

#include "CoreMinimal.h"
#include "IPlatformFilePak.h"
#include "VWLevelManager.generated.h"

UCLASS()
class VIRTUALWORLDMANAGER_API UVWLevelManager : public UObject
{
public:
    GENERATED_BODY()

    bool mountPakFromPath(const FString& pak_path);

    void getAllMapsInPak(TArray<FString>& map_list);
};
