// Copyright Epic Games, Inc. All Rights Reserved.

#include "InteriorSimBridge.h"

#include <Engine/Engine.h>
#include <Engine/World.h>
#include <EngineUtils.h>

#include <SimpleVehicle/SimModeSimpleVehicle.h>
#include <UrdfBot/SimModeUrdfBot.h>

#include "SimpleVehicleBrain.h"
#include "UrdfBotBrain.h"

#define LOCTEXT_NAMESPACE "FInteriorSimBridgeModule"

void FInteriorSimBridgeModule::StartupModule()
{
    // required to get updated gameworld instance and to add
    // ActorSpawnedEventHandler
    PostWorldInitializationDelegateHandle =
        FWorldDelegates::OnPostWorldInitialization.AddRaw(
            this,
            &FInteriorSimBridgeModule::PostWorldInitializationEventHandler);

    // required to reset any custom logic during a world cleanup
    WorldCleanupDelegateHandle = FWorldDelegates::OnWorldCleanup.AddRaw(
        this, &FInteriorSimBridgeModule::WorldCleanupEventHandler);
}

void FInteriorSimBridgeModule::PostWorldInitializationEventHandler(
    UWorld* InWorld, const UWorld::InitializationValues)
{
    check(InWorld);

    if (InWorld->IsGameWorld())
    {
        // make sure that local World reference is not in use
        check(!World);

        World = InWorld;

        // required to handle cases when new actors of custom classes are
        // spawned
        ActorSpawnedDelegateHandle = World->AddOnActorSpawnedHandler(
            FOnActorSpawned::FDelegate::CreateRaw(
                this, &FInteriorSimBridgeModule::ActorSpawnedEventHandler));
    }
}

void FInteriorSimBridgeModule::WorldCleanupEventHandler(UWorld* InWorld,
                                                        bool bSessionEnded,
                                                        bool bCleanupResources)
{
    check(InWorld);

    if (InWorld->IsGameWorld())
    {
        // make sure we are cleaning up based on the correct world
        check(World == InWorld);

        // remove event handlers bound to this world before world gets cleaned
        // up
        World->RemoveOnActorSpawnedHandler(ActorSpawnedDelegateHandle);
        ActorSpawnedDelegateHandle.Reset();

        // clear local reference to world as it will get cleaned up soon
        World = nullptr;
    }
}

void FInteriorSimBridgeModule::ActorSpawnedEventHandler(AActor* InActor)
{
    check(InActor);

    // check UrdfBot factory and create corresponding UBrain component
    // skip if Actor already contains UBrain component
    if (InActor->IsA(ASimModeUrdfBot::StaticClass()) &&
        !InActor->GetComponentByClass(UBrain::StaticClass()))
    {
        UUrdfBotBrain* Brain = NewObject<UUrdfBotBrain>(
            InActor, UUrdfBotBrain::StaticClass(), FName("UUrdfBotBrain"));

        check(Brain);
        Brain->RegisterComponent();
    }
    else if (InActor->IsA(ASimModeSimpleVehicle::StaticClass()) &&
             !InActor->GetComponentByClass(UBrain::StaticClass()))
    {
        USimpleVehicleBrain* Brain = NewObject<USimpleVehicleBrain>(
            InActor, USimpleVehicleBrain::StaticClass(),
            FName("USimpleVehicleBrain"));

        check(Brain);
        Brain->RegisterComponent();
    }
}

void FInteriorSimBridgeModule::ShutdownModule()
{
    // remove event handlers used by this module
    FWorldDelegates::OnPostWorldInitialization.Remove(
        PostWorldInitializationDelegateHandle);
    PostWorldInitializationDelegateHandle.Reset();

    FWorldDelegates::OnWorldCleanup.Remove(WorldCleanupDelegateHandle);
    WorldCleanupDelegateHandle.Reset();

    // local World will no longer be used, so remove clear it
    World = nullptr;
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FInteriorSimBridgeModule, InteriorSimBridge)
