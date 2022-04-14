#pragma once

#include "CoreMinimal.h"
#include "IPlatformFilePak.h"
#include "VWLevelManager.generated.h"

UCLASS()
class VIRTUALWORLDMANAGER_API AVWLevelManager : public AActor
{
public:
    GENERATED_BODY()

    bool MountPakFromPath(const FString& PakPath);

    void GetAllMapsInPak(TArray<FString>& MapList);
};