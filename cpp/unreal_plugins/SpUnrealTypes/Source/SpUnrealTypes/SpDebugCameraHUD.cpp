//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpDebugCameraHUD.h"

#include <Components/ActorComponent.h>
#include <Components/PrimitiveComponent.h>
#include <DrawDebugHelpers.h>   // ENABLE_DRAW_DEBUG
#include <Engine/ActorInstanceHandle.h>
#include <Engine/Canvas.h>
#include <Engine/DebugCameraHUD.h>
#include <Engine/Engine.h>      // GEngine
#include <Engine/EngineTypes.h> // FFontRenderInfo
#include <Engine/Font.h>
#include <GameFramework/Actor.h>

#include "SpCore/Unreal.h"

#include "SpUnrealTypes/SpDebugCameraController.h"

void ASpDebugCameraHUD::PostRender()
{
    ADebugCameraHUD::PostRender();

    #if ENABLE_DRAW_DEBUG
        if (bShowHUD) {
            ASpDebugCameraController* sp_debug_camera_controller = Cast<ASpDebugCameraController>(PlayerOwner);
            if (sp_debug_camera_controller && sp_debug_camera_controller->PlayerCameraManager) {
                FVector camera_location = sp_debug_camera_controller->PlayerCameraManager->GetCameraLocation();
                FRotator camera_rotation = sp_debug_camera_controller->PlayerCameraManager->GetCameraRotation();

                bool trace_complex = true;
                #if !(UE_BUILD_SHIPPING || UE_BUILD_TEST) // defined by the Unreal Build Tool
                    IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("g.DebugCameraTraceComplex"));
                    SP_ASSERT(cvar);
                    trace_complex = cvar->GetBool();
                #endif

                FCollisionQueryParams collision_query_params = FCollisionQueryParams(NAME_None, FCollisionQueryParams::GetUnknownStatId(), trace_complex, this);
                collision_query_params.bReturnPhysicalMaterial = true;
                FHitResult hit_result;
                bool hit = GetWorld()->LineTraceSingleByChannel(hit_result, camera_location, camera_rotation.Vector() * 100000.0f + camera_location, ECC_Visibility, collision_query_params);

                if (hit) {
                    UActorComponent* component = hit_result.Component.Get();
                    SP_ASSERT(component);
                    AActor* actor = component->GetOwner();
                    SP_ASSERT(actor);

                    UFont* font = GEngine->GetMediumFont();
                    FFontRenderInfo font_render_info = Canvas->CreateFontRenderInfo(false, true);

                    float x_single_line, y_single_line;
                    Canvas->StrLen(font, Unreal::toFString(" "), x_single_line, y_single_line);

                    float x = FMath::FloorToFloat(Canvas->SizeX*0.05f);
                    float y = FMath::FloorToFloat(Canvas->SizeY*0.9f);

                    Canvas->SetDrawColor(64, 64, 255, 255);
                    Canvas->DrawText(font, Unreal::toFString("SPEAR Debug Camera"), x, y, 1.0f, 1.0f, font_render_info);

                    Canvas->SetDrawColor(200, 200, 128, 255);

                    y += y_single_line;

                    if (Unreal::hasStableName(actor)) {
                        Canvas->DrawText(font, Unreal::toFString("Actor (stable Name): " + Unreal::getStableName(actor)), x, y, 1.0f, 1.0f, font_render_info);
                    } else {
                        Canvas->DrawText(font, Unreal::toFString("Actor doesn't have a stable name."), x, y, 1.0f, 1.0f, font_render_info);                        
                    }

                    y += y_single_line;
                    Canvas->DrawText(font, Unreal::toFString("Component (stable name): " + Unreal::getStableName(component)), x, y, 1.0f, 1.0f, font_render_info);
                }
            }
        }
    #endif
}
