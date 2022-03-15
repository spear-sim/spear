// Copyright Epic Games, Inc. All Rights Reserved.

#include "SimulationController.h"

#include <Engine/Engine.h>
#include <Engine/World.h>
#include <EngineUtils.h>

//#include <SimpleVehicle/SimModeSimpleVehicle.h>
//#include <UrdfBot/SimModeUrdfBot.h>

#include "Assert.h"

#define LOCTEXT_NAMESPACE "FSimulationControllerModule"

void FSimulationControllerModule::StartupModule()
{
    // required to add ActorSpawnedEventHandler
    PostWorldInitializationDelegateHandle =
        FWorldDelegates::OnPostWorldInitialization.AddRaw(
            this,
            &FSimulationControllerModule::PostWorldInitializationEventHandler);

    // required to reset any custom logic during a world cleanup
    WorldCleanupDelegateHandle = FWorldDelegates::OnWorldCleanup.AddRaw(
        this, &FSimulationControllerModule::WorldCleanupEventHandler);


    WorldInitializedActorsDelegateHandle =
        FWorldDelegates::OnWorldInitializedActors.AddRaw(
            this, &FSimulationController::WorldInitializedActorsEventHandler);

    WorldBeginPlayDelegateHandle = FWorldDelegates::OnWorldBeginPlay.AddRaw(this, &FSimulationController::WorldBeginPlayEventHandler);

}

void FSimulationControllerModule::ShutdownModule()
{
    // If this module is unloaded in the middle of simulation for some reason,
    // raise an error because we do not support this and we want to know when
    // this happens
    ASSERT(!ActorSpawnedDelegateHandle.IsValid());

    // remove event handlers used by this module
    FWorldDelegates::OnWorldCleanup.Remove(WorldCleanupDelegateHandle);
    WorldCleanupDelegateHandle.Reset();

    FWorldDelegates::OnPostWorldInitialization.Remove(
        PostWorldInitializationDelegateHandle);
    PostWorldInitializationDelegateHandle.Reset();
}

void FSimulationControllerModule::PostWorldInitializationEventHandler(
    UWorld* World, const UWorld::InitializationValues)
{
    ASSERT(World);

    if (World->IsGameWorld())
    {
        // required to handle cases when new actors of custom classes are
        // spawned
        ActorSpawnedDelegateHandle = World->AddOnActorSpawnedHandler(
            FOnActorSpawned::FDelegate::CreateRaw(
                this, &FSimulationControllerModule::ActorSpawnedEventHandler));
    }
}

void FSimulationControllerModule::WorldCleanupEventHandler(UWorld* World,
                                                        bool bSessionEnded,
                                                        bool bCleanupResources)
{
    ASSERT(World);

    if (World->IsGameWorld())
    {
        // remove event handlers bound to this world before world gets cleaned
        // up
        World->RemoveOnActorSpawnedHandler(ActorSpawnedDelegateHandle);
        ActorSpawnedDelegateHandle.Reset();
    }
}


void FSimulationControllerModule::WorldInitializedActorsEventHandler(const FActorsInitializedParams& Params)
{
    UE_LOG(LogTemp, Warning, TEXT(" "));
    UE_LOG(LogTemp, Warning, TEXT("OnWorldInitializedActors called...."));
    UE_LOG(LogTemp, Warning, TEXT(" "));
}

void FSimulationControllerModule::WorldBeginPlayEventHandler()
{
    UE_LOG(LogTemp, Warning, TEXT(" "));
    UE_LOG(LogTemp, Warning, TEXT("OnWorldBeginPlay called...."));
    UE_LOG(LogTemp, Warning, TEXT(" "));
}

void FSimulationControllerModule::ActorSpawnedEventHandler(AActor* InActor)
    {
    ASSERT(InActor);

    // // ASSERT UrdfBot factory and create corresponding UBrain component
    // // skip if Actor already contains UBrain component
    // if (InActor->IsA(ASimModeUrdfBot::StaticClass()) &&
    //     !InActor->GetComponentByClass(UBrain::StaticClass()))
    // {
    //     UUrdfBotBrain* Brain = NewObject<UUrdfBotBrain>(
    //         InActor, UUrdfBotBrain::StaticClass(), FName("UUrdfBotBrain"));

    //     ASSERT(Brain);
    //     Brain->RegisterComponent();
    // }
    // else if (InActor->IsA(ASimModeSimpleVehicle::StaticClass()) &&
    //          !InActor->GetComponentByClass(UBrain::StaticClass()))
    // {
    //     USimpleVehicleBrain* Brain = NewObject<USimpleVehicleBrain>(
    //         InActor, USimpleVehicleBrain::StaticClass(),
    //         FName("USimpleVehicleBrain"));

    //     ASSERT(Brain);
    //     Brain->RegisterComponent();
    // }
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FSimulationControllerModule, SimulationController)
