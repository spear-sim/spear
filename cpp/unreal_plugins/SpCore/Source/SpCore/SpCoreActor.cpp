//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpCoreActor.h"

#include <Containers/Array.h>
#include <Engine/Engine.h>          // GEngine
#include <Engine/EngineBaseTypes.h> // ETickingGroup
#include <Engine/EngineTypes.h>     // FHitResult
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>           // uint64
#include <Kismet/GameplayStatics.h>
#include <Math/Vector.h>
#include <Misc/CoreDelegates.h>
#include <UObject/NameTypes.h>      // FName

#include "SpCore/Assert.h"
#include "SpCore/CppFuncData.h"
#include "SpCore/CppFuncComponent.h"
#include "SpCore/Log.h"
#include "SpCore/StableNameComponent.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealObj.h"

ASpCoreActor::ASpCoreActor()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = true; // we want to update IsGamePaused state when paused
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;

    StableNameComponent = Unreal::createComponentInsideOwnerConstructor<UStableNameComponent>(this, "stable_name");
    SP_ASSERT(StableNameComponent);

    CppFuncComponent = Unreal::createComponentInsideOwnerConstructor<UCppFuncComponent>(this, "cpp_func");
    SP_ASSERT(CppFuncComponent);

    // TODO: remove these CppFuncs because they are only here for debugging

    CppFuncComponent->registerFunc("hello_world", [](const CppFuncComponentArgs& args) -> CppFuncComponentReturnValues {

        CppFuncData<uint8_t> hello("hello");
        hello.setData("Hello!"); // initialize from string literal

        CppFuncData<uint8_t> world("world");
        std::string world_str = "World!";
        world.setData(world_str); // or from string

        CppFuncData<double> data("data");
        data.setData({1.1, 2.2, 4.4, 8.8}); // or from initializer list of double

        // NOTE: for maximum efficiency, getReturnValuesFromData(...) performs std::move operations and invalidates data objects
        CppFuncComponentReturnValues return_values;
        return_values.return_values_ = CppFuncDataUtils::getReturnValuesFromData({hello.getPtr(), world.getPtr(), data.getPtr()});

        return return_values;
    });

    CppFuncComponent->registerFunc("my_func", [](const CppFuncComponentArgs& args) -> CppFuncComponentReturnValues {

        // create data objects directly from arg data
        CppFuncData<double> location("location");
        CppFuncData<double> rotation("rotation");
        CppFuncDataUtils::setDataFromArgs({location.getPtr(), rotation.getPtr()}, args.args_);

        // create new data objects
        CppFuncData<double> new_location("new_location");
        CppFuncData<double> new_rotation("new_rotation");
        new_location.setData(location.getData() | std::views::transform([](auto x) { return 2.0*x; })); // initialize from range of double
        new_rotation.setData(rotation.getData() | std::views::transform([](auto x) { return 3.0*x; })); // initialize from range of double

        // set return data from data objects (NOTE: getReturnValuesFromData(...) performs std::move operations and invalidates data objects)
        CppFuncComponentReturnValues return_values;
        return_values.return_values_ = CppFuncDataUtils::getReturnValuesFromData({new_location.getPtr(), new_rotation.getPtr()});


        return return_values;
    });
}

ASpCoreActor::~ASpCoreActor()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpCoreActor::Tick(float delta_time)
{
    AActor::Tick(delta_time);

    IsGamePaused = UGameplayStatics::IsGamePaused(GetWorld());

    actor_hit_event_descs_.Empty();
}

void ASpCoreActor::PauseGame()
{
    UGameplayStatics::SetGamePaused(GetWorld(), true);
}

void ASpCoreActor::UnpauseGame()
{
    UGameplayStatics::SetGamePaused(GetWorld(), false);
}

void ASpCoreActor::ToggleGamePaused()
{
    UGameplayStatics::SetGamePaused(GetWorld(), !UGameplayStatics::IsGamePaused(GetWorld()));
}

void ASpCoreActor::SubscribeToActorHitEvents(AActor* actor)
{
    actor->OnActorHit.AddDynamic(this, &ASpCoreActor::ActorHitHandler);
}

void ASpCoreActor::UnsubscribeFromActorHitEvents(AActor* actor)
{
    actor->OnActorHit.AddDynamic(this, &ASpCoreActor::ActorHitHandler);
}

TArray<FActorHitEventDesc> ASpCoreActor::GetActorHitEventDescs()
{
    return actor_hit_event_descs_;
}

void ASpCoreActor::ActorHitHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result)
{
    SP_ASSERT(self_actor);
    SP_ASSERT(other_actor);

    FActorHitEventDesc actor_hit_event_desc;
    actor_hit_event_desc.SelfActor = reinterpret_cast<uint64>(self_actor);
    actor_hit_event_desc.OtherActor = reinterpret_cast<uint64>(other_actor);
    actor_hit_event_desc.NormalImpulse = normal_impulse;
    actor_hit_event_desc.HitResult = hit_result;

    actor_hit_event_desc.SelfActorDebugPtr = Unreal::toFString(Std::toStringFromPtr(self_actor));
    actor_hit_event_desc.SelfActorDebugInfo = Unreal::toFString(Unreal::getObjectPropertiesAsString(self_actor));
    actor_hit_event_desc.OtherActorDebugPtr = Unreal::toFString(Std::toStringFromPtr(other_actor));
    actor_hit_event_desc.OtherActorDebugInfo = Unreal::toFString(Unreal::getObjectPropertiesAsString(other_actor));

    actor_hit_event_descs_.Add(actor_hit_event_desc);

    // HACK: Strictly speaking, this code doesn't need to be here, but I wanted to test this function when the array of hit events is non-empty.
    UFunction* ufunction = Unreal::findFunctionByName(this->GetClass(), "GetActorHitEventDescs");
    std::map<std::string, std::string> return_values = Unreal::callFunction(this, ufunction);
    SP_LOG(return_values.at("ReturnValue"));
}

#if WITH_EDITOR // defined in an auto-generated header
    void ASpCoreActor::PostActorCreated()
    { 
        AActor::PostActorCreated();
        initializeActorLabelHandlers();
    }

    void ASpCoreActor::PostLoad()
    {
        AActor::PostLoad();
        initializeActorLabelHandlers();
    }

    void ASpCoreActor::BeginDestroy()
    {
        AActor::BeginDestroy();
        requestTerminateActorLabelHandlers();
    }

    void ASpCoreActor::initializeActorLabelHandlers()
    {
        SP_ASSERT(GEngine);
        SP_ASSERT(!actor_label_changed_handle_.IsValid());
        SP_ASSERT(!level_actor_folder_changed_handle_.IsValid());
        actor_label_changed_handle_ = FCoreDelegates::OnActorLabelChanged.AddUObject(this, &ASpCoreActor::actorLabelChangedHandler);
        level_actor_folder_changed_handle_ = GEngine->OnLevelActorFolderChanged().AddUObject(this, &ASpCoreActor::levelActorFolderChangedHandler);
    }

    void ASpCoreActor::requestTerminateActorLabelHandlers()
    {
        // Need to check IsValid() here because BeginDestroy() is called for default objects, but PostActorCreated() and PostLoad() are not.

        if (level_actor_folder_changed_handle_.IsValid()) {
            SP_ASSERT(GEngine);
            GEngine->OnLevelActorFolderChanged().Remove(level_actor_folder_changed_handle_);
            level_actor_folder_changed_handle_.Reset();
        }

        if (actor_label_changed_handle_.IsValid()) {
            FCoreDelegates::OnActorLabelChanged.Remove(actor_label_changed_handle_);
            actor_label_changed_handle_.Reset();
        }
    }

    void ASpCoreActor::actorLabelChangedHandler(AActor* actor)
    {
        SP_ASSERT(actor);
        Unreal::requestUpdateStableName(actor);
    }

    void ASpCoreActor::levelActorFolderChangedHandler(const AActor* actor, FName name)
    {
        SP_ASSERT(actor);
        Unreal::requestUpdateStableName(actor);
    }
#endif
