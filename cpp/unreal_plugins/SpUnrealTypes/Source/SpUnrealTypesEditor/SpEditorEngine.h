//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/Array.h>     // TArray
#include <Editor.h>               // GEditor
#include <Kismet/BlueprintFunctionLibrary.h>
#include <LevelEditorViewport.h>  // FLevelEditorViewportClient, GCurrentLevelEditingViewportClient
#include <Math/IntPoint.h>        // FIntPoint
#include <Math/Rotator.h>         // FRotator
#include <Math/Vector.h>          // FVector
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY, USTRUCT
#include <UnrealClient.h>         // FViewportClient

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
    float OrthoWidth = 0.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FIntPoint ViewportSize = FIntPoint::ZeroValue;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    bool bIsCurrentLevelEditing = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    bool bIsActive = false;
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
        SP_ASSERT(GEditor->GetActiveViewport());
        FViewportClient* active_viewport_client = GEditor->GetActiveViewport()->GetClient();
        SP_ASSERT(active_viewport_client);

        TArray<FSpLevelViewportClientDesc> level_viewport_client_descs;
        for (auto level_editor_viewport_client : GEditor->GetLevelViewportClients()) {
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

            // OrthoZoom is not a world-space width, so we compute the full world-space width of the viewport the
            // same way as FEditorViewportClient::CalcSceneView(...). This width matches the semantics of
            // UCameraComponent::OrthoWidth.
            desc.OrthoWidth = level_editor_viewport_client->GetOrthoUnitsPerPixel(level_editor_viewport_client->Viewport)*level_editor_viewport_client->Viewport->GetSizeXY().X;

            desc.ViewportSize = level_editor_viewport_client->Viewport->GetSizeXY();
            desc.bIsCurrentLevelEditing = level_editor_viewport_client == GCurrentLevelEditingViewportClient;
            desc.bIsActive = level_editor_viewport_client == active_viewport_client;
            level_viewport_client_descs.Add(desc);
        }

        return level_viewport_client_descs;
    }
};
