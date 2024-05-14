//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpCoreActor.h"

#include <memory> // std::make_unique
#include <string>
#include <vector>

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
#include "SpCore/CppFunc.h"
#include "SpCore/CppFuncComponent.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemoryRegion.h"
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

    // TODO: remove CppFuncComponent because it is only here for debugging
    CppFuncComponent = Unreal::createComponentInsideOwnerConstructor<UCppFuncComponent>(this, "cpp_func");
    SP_ASSERT(CppFuncComponent);

    initializeCppFuncs();
}

ASpCoreActor::~ASpCoreActor()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpCoreActor::BeginDestroy()
{
    AActor::BeginDestroy();
    terminateCppFuncs();
    #if WITH_EDITOR
        requestTerminateActorLabelHandlers();
    #endif
}

void ASpCoreActor::Tick(float delta_time)
{
    AActor::Tick(delta_time);
    IsGamePaused = UGameplayStatics::IsGamePaused(GetWorld());
    actor_hit_event_descs_.Empty();
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
#endif

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

void ASpCoreActor::initializeCppFuncs()
{
    int shared_memory_num_bytes = 1024;
    shared_memory_region_ = std::make_unique<SharedMemoryRegion>(shared_memory_num_bytes);
    SP_ASSERT(shared_memory_region_);

    std::string shared_memory_name = "my_shared_memory";
    CppFuncSharedMemoryView shared_memory_view(shared_memory_region_->getView(), CppFuncSharedMemoryUsageFlags::Arg | CppFuncSharedMemoryUsageFlags::ReturnValue);
    CppFuncComponent->registerSharedMemoryView(shared_memory_name, shared_memory_view);

    CppFuncComponent->registerFunc("hello_world", [this](CppFuncPackage& args) -> CppFuncPackage {

        // CppFuncData objects are {named, strongly typed} arrays that can be efficiently passed
        // {to, from} Python.
        CppFuncData<uint8_t> hello("hello");
        hello.setData("Hello World!");

        // Get some some data from the Unreal Engine.
        AActor* actor = Unreal::findActorByName(GetWorld(), "SpCore/SpCoreActor");
        FVector location = actor->GetActorLocation();

        // Store the Unreal data in a CppFuncData object to efficiently return it to Python.
        // Here we use shared memory for the most efficient possible communication.
        CppFuncData<double> unreal_data("unreal_data");
        unreal_data.setData("my_shared_memory", shared_memory_region_->getView(), 3);
        unreal_data.setValues({location.X, location.Y, location.Z});

        // Return CppFuncData objects.
        CppFuncPackage return_values;
        return_values.items_ = CppFuncUtils::moveDataToItems({hello.getPtr(), unreal_data.getPtr()});
        return return_values;
    });

    CppFuncComponent->registerFunc("my_func", [this](CppFuncPackage& args) -> CppFuncPackage {

        // create view objects directly from arg data
        CppFuncView<double> location("location");
        CppFuncView<double> rotation("rotation");
        CppFuncUtils::setViewsFromItems({location.getPtr(), rotation.getPtr()}, args.items_);

        // create new data objects
        CppFuncData<double> new_location("new_location");
        CppFuncData<double> new_rotation("new_rotation");
        new_location.setData(location.getView() | std::views::transform([](auto x) { return 2.0*x; })); // initialize from range of double
        new_rotation.setData(rotation.getView() | std::views::transform([](auto x) { return 3.0*x; })); // initialize from range of double

        // create new data objects backed by shared memory
        CppFuncData<float> my_floats("my_floats");
        my_floats.setData("my_shared_memory", shared_memory_region_->getView(), 3);
        my_floats.setValues({99.0, 98.0, 97.0});

        CppFuncPackage return_values;
        return_values.items_ = CppFuncUtils::moveDataToItems({new_location.getPtr(), new_rotation.getPtr(), my_floats.getPtr()});

        return return_values;
    });
}

void ASpCoreActor::terminateCppFuncs()
{
    CppFuncComponent->unregisterFunc("my_func");
    CppFuncComponent->unregisterFunc("hello_world");

    CppFuncComponent->unregisterSharedMemoryView("my_shared_memory");
    SP_ASSERT(shared_memory_region_);
    shared_memory_region_ = nullptr;
}

#if WITH_EDITOR
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
