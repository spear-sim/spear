//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Editor.h>              // GEditor
#include <Kismet/BlueprintFunctionLibrary.h>
#include <LevelEditorViewport.h> // FLevelEditorViewportClient

#include "SpCore/Assert.h"

#include "SpEditorEngine.generated.h"

USTRUCT(BlueprintType)
struct FSpLevelViewportClientDesc
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    bool bIsPerspective = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FVector CameraLocation = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FRotator CameraRotation = FRotator::ZeroRotator;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    float ViewFOV = 0.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    float FOVAngle = 0.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    float AspectRatio = 0.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    float OrthoZoom = 0.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FIntPoint ViewportSize = FIntPoint::ZeroValue;
};

UCLASS()
class USpEditorEngine : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static TArray<FSpLevelViewportClientDesc> GetLevelViewportClients()
    {
        SP_ASSERT(GEditor);

        TArray<FSpLevelViewportClientDesc> level_viewport_client_descs;
        for (FLevelEditorViewportClient* level_editor_viewport_client : GEditor->GetLevelViewportClients()) {
            SP_ASSERT(level_editor_viewport_client);
            SP_ASSERT(level_editor_viewport_client->Viewport);

            FSpLevelViewportClientDesc desc;
            desc.bIsPerspective = level_editor_viewport_client->IsPerspective();
            desc.CameraLocation = level_editor_viewport_client->GetViewLocation();
            desc.CameraRotation = level_editor_viewport_client->GetViewRotation();
            desc.ViewFOV = level_editor_viewport_client->ViewFOV;
            desc.FOVAngle = level_editor_viewport_client->FOVAngle;
            desc.AspectRatio = level_editor_viewport_client->AspectRatio;
            desc.OrthoZoom = level_editor_viewport_client->GetOrthoZoom();
            desc.ViewportSize = level_editor_viewport_client->Viewport->GetSizeXY();

            level_viewport_client_descs.Add(desc);
        }

        return level_viewport_client_descs;
    }
};
