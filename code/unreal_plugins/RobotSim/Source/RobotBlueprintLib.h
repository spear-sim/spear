// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <regex>
#include <string>

#include "common_utils/RobotSimSettings.hpp"
#include "Components/InputComponent.h"
#include "Components/MeshComponent.h"
#include "CoreMinimal.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerInput.h"
#include "IImageWrapperModule.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetStringLibrary.h"
#include "LandscapeProxy.h"
#include "ProceduralMeshComponent.h"
#include "Runtime/Foliage/Public/FoliageType.h"
#include "Runtime/Landscape/Classes/LandscapeComponent.h"

#include "RobotBlueprintLib.generated.h"


UENUM(BlueprintType)
enum class LogDebugLevel : uint8 {
    Informational UMETA(DisplayName = "Informational"),
    Success UMETA(DisplayName = "Success"),
    Failure UMETA(DisplayName = "Failure"),
    Unimportant UMETA(DisplayName = "Unimportant")
};

/**
 * @brief
 *
 */
UCLASS()
class URobotBlueprintLib : public UBlueprintFunctionLibrary {
    GENERATED_BODY()

public:
    /**
     * @brief
     *
     */
    static void OnBeginPlay();

    /**
     * @brief
     *
     */
    static void OnEndPlay();

    /**
     * @brief
     *
     * @param prefix
     * @param suffix
     * @param level
     * @param persist_sec
     */
    static void LogMessageString(const std::string& prefix,
                                 const std::string& suffix,
                                 LogDebugLevel level,
                                 float persist_sec = 60);

    /**
     * @brief
     *
     * @param prefix
     * @param suffix
     * @param level
     * @param persist_sec
     */
    UFUNCTION(BlueprintCallable, Category = "Utils")
    static void LogMessage(const FString& prefix,
                           const FString& suffix,
                           LogDebugLevel level,
                           float persist_sec = 60);

    /**
     * @brief Get the World To Meters Scale object
     *
     * @param context
     * @return float
     */
    static float GetWorldToMetersScale(const AActor* context);

    /**
     * @brief Get the Actor Component object
     *
     * @tparam T
     * @param actor
     * @param name
     * @return T*
     */
    template <typename T>
    static T* GetActorComponent(AActor* actor,
                                FString name);

    /**
     * @brief
     *
     * @tparam T
     * @param context
     * @param name
     * @return T*
     */
    template <typename T>
    static T* FindActor(const UObject* context,
                        FString name)
    {
        TArray<AActor*> foundActors;
        FindAllActor<T>(context, foundActors);
        FName name_n = FName(*name);

        for (AActor* actor : foundActors) {
            if (actor->ActorHasTag(name_n) or actor->GetName().Compare(name) == 0) {
                return static_cast<T*>(actor);
            }
        }

        // URobotBlueprintLib::LogMessage(name + TEXT(" Actor not found!"), TEXT(""), LogDebugLevel::Failure);

        return nullptr;
    }

    /**
     * @brief
     *
     * @tparam T
     * @param context
     * @param foundActors
     */
    template <typename T>
    static void FindAllActor(const UObject* context, TArray<AActor*>& foundActors)
    {
        UGameplayStatics::GetAllActorsOfClass(context, T::StaticClass(), foundActors);
    }

    /**
     * @brief
     *
     * @param actor
     * @param start
     * @param end
     * @param ignore_actor
     * @param collision_channel
     * @return true
     * @return false
     */
    static bool HasObstacle(const AActor* actor,
                            const FVector& start,
                            const FVector& end,
                            const AActor* ignore_actor = nullptr,
                            ECollisionChannel collision_channel = ECC_Visibility);

    /**
     * @brief Get the Obstacle object
     *
     * @param actor
     * @param start
     * @param end
     * @param hit
     * @param ignore_actors
     * @param collision_channel
     * @param ignore_root_actor
     * @return true
     * @return false
     */
    static bool GetObstacle(const AActor* actor,
                            const FVector& start,
                            const FVector& end, FHitResult& hit,
                            TArray<const AActor*> ignore_actors = TArray<const AActor*>(),
                            ECollisionChannel collision_channel = ECC_Visibility,
                            bool ignore_root_actor = true);

    /**
     * @brief Get the Last Obstacle Position object
     *
     * @param actor
     * @param start
     * @param end
     * @param hit
     * @param ignore_actor
     * @param collision_channel
     * @return true
     * @return false
     */
    static bool GetLastObstaclePosition(const AActor* actor,
                                        const FVector& start,
                                        const FVector& end,
                                        FHitResult& hit,
                                        const AActor* ignore_actor = nullptr,
                                        ECollisionChannel collision_channel = ECC_Visibility);

    /**
     * @brief
     *
     * @param follower
     * @param followee
     * @param offset
     * @param fixed_z
     * @param fixed_z_val
     */
    static void FollowActor(AActor* follower,
                            const AActor* followee,
                            const FVector& offset,
                            bool fixed_z = false,
                            float fixed_z_val = 2.0f);

    /**
     * @brief Set the Mesh Stencil I D object
     *
     * @param mesh_name
     * @param object_id
     * @param is_name_regex
     * @return true
     * @return false
     */
    static bool SetMeshStencilID(const std::string& mesh_name,
                                 int object_id,
                                 bool is_name_regex = false);

    /**
     * @brief Get the Mesh Stencil I D object
     *
     * @param mesh_name
     * @return int
     */
    static int GetMeshStencilID(const std::string& mesh_name);

    /**
     * @brief
     *
     * @param ignore_existing
     */
    static void InitializeMeshStencilIDs(bool ignore_existing);

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    static bool IsInGameThread();

    /**
     * @brief Get the Mesh Name object
     *
     * @tparam T
     * @param mesh
     * @return std::string
     */
    template <class T>
    static std::string GetMeshName(T* mesh)
    {
        switch (mesh_naming_method_) {
        case RobotSim::RobotSimSettings::SegmentationSetting::MeshNamingMethodType::OwnerName:
            if (mesh->GetOwner()) {
                // For foliage actors, need to use mesh's name in order to
                // differentiate different meshes from each other. Otherwise,
                // all items placed with the foliage actor will get the same ID.
                auto owner = mesh->GetOwner();
                if (owner->GetName().StartsWith("InstancedFoliageActor")) {
                    FString meshName = mesh->GetName();

                    // It is expected that the user will create a blueprint
                    // class that starts with "IFA_" for each of the classes
                    // they want to differentiate. Unreal adds a _C_ into the
                    // name for blueprint created classes, which messes up the
                    // hash. Remove it as well.
                    if (meshName.StartsWith(TEXT("IFA_"))) {
                        meshName = meshName.Replace(TEXT("IFA_"), TEXT("")).Replace(TEXT("_C_"), TEXT(""));
                    }

                    return std::string(TCHAR_TO_UTF8(*(meshName)));
                }

                return std::string(TCHAR_TO_UTF8(*(owner->GetName())));
            }
            else
                return ""; // std::string(TCHAR_TO_UTF8(*(UKismetSystemLibrary::GetDisplayName(mesh))));
        case RobotSim::RobotSimSettings::SegmentationSetting::MeshNamingMethodType::StaticMeshName:
            if (mesh) {
                return std::string(TCHAR_TO_UTF8(*(mesh->GetName())));
            }

            else {
                return "";
            }

        default:
            return "";
        }
    }

    /**
     * @brief Get the Mesh Name object
     *
     * @param mesh
     * @return std::string
     */
    static std::string GetMeshName(ALandscapeProxy* mesh);

    /**
     * @brief Get the Mesh Name object
     *
     * @param meshComponent
     * @return std::string
     */
    static std::string GetMeshName(UProceduralMeshComponent* meshComponent);

    /**
     * @brief
     *
     * @tparam UserClass
     * @param action_name
     * @param in_key
     * @param actor
     * @param func
     * @param on_press_or_release
     * @param shift_key
     * @param control_key
     * @param alt_key
     * @param command_key
     * @return FInputActionBinding&
     */
    template <class UserClass>
    static FInputActionBinding& BindActionToKey(const FName action_name,
                                                const FKey in_key,
                                                UserClass* actor,
                                                typename FInputActionHandlerSignature::TUObjectMethodDelegate<UserClass>::FMethodPtr func,
                                                bool on_press_or_release = false,
                                                bool shift_key = false,
                                                bool control_key = false,
                                                bool alt_key = false,
                                                bool command_key = false)
    {
        FInputActionKeyMapping action(action_name, in_key, shift_key, control_key, alt_key, command_key);

        APlayerController* controller = actor->GetWorld()->GetFirstPlayerController();

        controller->PlayerInput->AddActionMapping(action);
        return controller->InputComponent->BindAction(action_name, on_press_or_release ? IE_Pressed : IE_Released, actor, func);
    }

    /**
     * @brief
     *
     * @tparam UserClass
     * @param axis_name
     * @param in_key
     * @param actor
     * @param obj
     * @param func
     * @return FInputAxisBinding&
     */
    template <class UserClass>
    static FInputAxisBinding& BindAxisToKey(const FName axis_name,
                                            const FKey in_key,
                                            AActor* actor,
                                            UserClass* obj,
                                            typename FInputAxisHandlerSignature::TUObjectMethodDelegate<UserClass>::FMethodPtr func)
    {
        FInputAxisKeyMapping axis(axis_name, in_key);

        return URobotBlueprintLib::BindAxisToKey(axis, actor, obj, func);
    }

    /**
     * @brief
     *
     * @tparam UserClass
     * @param axis
     * @param actor
     * @param obj
     * @param func
     * @return FInputAxisBinding&
     */
    template <class UserClass>
    static FInputAxisBinding& BindAxisToKey(const FInputAxisKeyMapping& axis,
                                            AActor* actor,
                                            UserClass* obj,
                                            typename FInputAxisHandlerSignature::TUObjectMethodDelegate<UserClass>::FMethodPtr func)
    {
        APlayerController* controller = actor->GetWorld()->GetFirstPlayerController();

        controller->PlayerInput->AddAxisMapping(axis);
        return controller->InputComponent->BindAxis(axis.AxisName, obj, func);
    }

    /**
     * @brief
     *
     * @param axis
     * @param axis_binding
     * @param actor
     * @return int
     */
    static int RemoveAxisBinding(const FInputAxisKeyMapping& axis,
                                 FInputAxisBinding* axis_binding,
                                 AActor* actor);

    /**
     * @brief
     *
     * @param actor
     */
    static void EnableInput(AActor* actor);

    /**
     * @brief
     *
     * @param InFunction
     * @param wait
     * @param InStatId
     */
    static void RunCommandOnGameThread(TFunction<void()> InFunction,
                                       bool wait = false,
                                       const TStatId InStatId = TStatId());

    /**
     * @brief Get the Display Gamma object
     *
     * @return float
     */
    static float GetDisplayGamma();

    /**
     * @brief
     *
     * @param MessageType
     * @param message
     * @param title
     * @return EAppReturnType::Type
     */
    static EAppReturnType::Type ShowMessage(EAppMsgType::Type MessageType,
                                            const std::string& message,
                                            const std::string& title);

    /**
     * @brief Get the Log Messages Hidden object
     *
     * @return true
     * @return false
     */
    static bool getLogMessagesHidden()
    {
        return log_messages_hidden_;
    }

    /**
     * @brief Set the Log Messages Hidden object
     * 
     * @param is_hidden 
     */
    static void setLogMessagesHidden(bool is_hidden)
    {
        log_messages_hidden_ = is_hidden;
    }

    /**
     * @brief Set the Mesh Naming Method object
     * 
     * @param method 
     */
    static void SetMeshNamingMethod(RobotSim::RobotSimSettings::SegmentationSetting::MeshNamingMethodType method)
    {
        mesh_naming_method_ = method;
    }

    /**
     * @brief 
     * 
     * @param context 
     * @param enable 
     */
    static void enableWorldRendering(AActor* context,
                                     bool enable);

    /**
     * @brief 
     * 
     * @param context 
     * @param enable 
     */
    static void enableViewportRendering(AActor* context,
                                        bool enable);

    /**
     * @brief Set the Simulate Physics object
     * 
     * @param actor 
     * @param simulate_physics 
     */
    static void setSimulatePhysics(AActor* actor,
                                   bool simulate_physics);

    /**
     * @brief 
     * 
     * @param actor 
     */
    static void resetSimulatePhysics(AActor* actor);

    /**
     * @brief Get the Physics Components object
     * 
     * @param actor 
     * @return std::vector<UPrimitiveComponent*> 
     */
    static std::vector<UPrimitiveComponent*> getPhysicsComponents(AActor* actor);

    /**
     * @brief 
     * 
     * @param name 
     * @return UObject* 
     */
    static UObject* LoadObject(const std::string& name);

    /**
     * @brief 
     * 
     * @param name 
     * @return UClass* 
     */
    static UClass* LoadClass(const std::string& name);

    /**
     * @brief Get the Image Wrapper Module object
     * 
     * @return IImageWrapperModule* 
     */
    static IImageWrapperModule* getImageWrapperModule();

    /**
     * @brief Set the Unreal Clock Speed object
     * 
     * @param context 
     * @param clock_speed 
     */
    static void setUnrealClockSpeed(const AActor* context,
                                    float clock_speed);

    /**
     * @brief 
     * 
     * @param width 
     * @param height 
     * @param src 
     * @param dest 
     */
    static void CompressImageArray(int32 width,
                                   int32 height,
                                   const TArray<FColor>& src,
                                   TArray<uint8>& dest);

private:

    /**
     * @brief 
     * 
     * @tparam T 
     * @param mesh 
     * @param ignore_existing 
     */
    template <typename T>
    static void InitializeObjectStencilID(T* mesh, bool ignore_existing = true)
    {
        std::string mesh_name = common_utils::Utils::toLower(GetMeshName(mesh));

        if (mesh_name == "" or common_utils::Utils::startsWith(mesh_name, "default_")) {
            // common_utils::Utils::DebugBreak();
            return;
        }

        FString name(mesh_name.c_str());
        int hash = 5;

        for (int idx = 0; idx < name.Len(); ++idx) {
            auto char_num = UKismetStringLibrary::GetCharacterAsNumber(name, idx);

            if (char_num < 97) {
                continue; // numerics and other punctuations
            }

            hash += char_num;
        }

        if (ignore_existing || mesh->CustomDepthStencilValue == 0) { // if value is already set then don't bother
            SetObjectStencilID(mesh, hash % 256);
        }
    }

    /**
     * @brief 
     * 
     * @param mesh 
     * @param vv 
     * @param ignore_existing 
     */
    static void InitializeFoliageStencilID(UFoliageType* mesh,
                                           int vv,
                                           bool ignore_existing = true)
    {
        mesh->bRenderCustomDepth = true;
        mesh->CustomDepthStencilValue = vv;
    }

    /**
     * @brief Set the Object Stencil I D If Match object
     * 
     * @tparam T 
     * @param mesh 
     * @param object_id 
     * @param mesh_name 
     * @param is_name_regex 
     * @param name_regex 
     * @param changes 
     */
    template <typename T>
    static void SetObjectStencilIDIfMatch(T* mesh,
                                          int object_id,
                                          const std::string& mesh_name,
                                          bool is_name_regex,
                                          const std::regex& name_regex,
                                          int& changes)
    {
        std::string comp_mesh_name = GetMeshName(mesh);
        if (comp_mesh_name == "") {
            return;
        }

        bool is_match = (!is_name_regex and (comp_mesh_name == mesh_name or comp_mesh_name == (mesh_name + "_visual"))) or (is_name_regex and std::regex_match(comp_mesh_name, name_regex));

        if (is_match) {
            ++changes;
            SetObjectStencilID(mesh, object_id);
        }
    }

    /**
     * @brief Set the Object Stencil I D object
     * 
     * @tparam T 
     * @param mesh 
     * @param object_id 
     */
    template <typename T>
    static void SetObjectStencilID(T* mesh, int object_id)
    {
        if (object_id < 0) {
            mesh->SetRenderCustomDepth(false);
        }
        else {
            mesh->SetCustomDepthStencilValue(object_id);
            mesh->SetRenderCustomDepth(true);
        }
        // mesh->SetVisibility(false);
        // mesh->SetVisibility(true);
    }

    /**
     * @brief Set the Object Stencil I D object
     * 
     * @param mesh 
     * @param object_id 
     */
    static void SetObjectStencilID(ALandscapeProxy* mesh, int object_id)
    {
        if (object_id < 0) {
            mesh->bRenderCustomDepth = false;
        }
        else {
            mesh->CustomDepthStencilValue = object_id;
            mesh->bRenderCustomDepth = true;
        }

        // Explicitly set the custom depth state on the components so the
        // render state is marked dirty and the update actually takes effect
        // immediately.
        for (ULandscapeComponent* comp : mesh->LandscapeComponents) {
            if (object_id < 0) {
                comp->SetRenderCustomDepth(false);
            }
            else {
                comp->SetCustomDepthStencilValue(object_id);
                comp->SetRenderCustomDepth(true);
            }
        }
    }

    /**
     * @brief 
     * 
     * @param uncompressed 
     * @param width 
     * @param height 
     * @param compressed 
     * @return true 
     * @return false 
     */
    static bool CompressUsingImageWrapper(const TArray<uint8>& uncompressed,
                                          const int32 width,
                                          const int32 height,
                                          TArray<uint8>& compressed);

private:
    static bool log_messages_hidden_;
    // FViewPort doesn't expose this field so we are doing dirty work around by maintaining count by ourselves
    static uint32_t flush_on_draw_count_;
    static RobotSim::RobotSimSettings::SegmentationSetting::MeshNamingMethodType mesh_naming_method_;
    static IImageWrapperModule* image_wrapper_module_;
};
