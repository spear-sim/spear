#pragma once

#include "CoreMinimal.h"
#include "IPlatformFilePak.h"
#include "VWLevelManager.generated.h"

enum EPostProcessMaterialType
{
    Default = 0,
	Semantic,
    Diffuse,
    CelShader,
    Painter
};

UCLASS()
class VIRTUALWORLDMANAGER_API AVWLevelManager : public AActor
{
public:
    GENERATED_BODY()
    AVWLevelManager();

    bool mountPakFromPath(const FString& pak_path);

    void getAllMapsInPak(TArray<FString>& map_list);
};
