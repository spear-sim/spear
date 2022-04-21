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

void FInteriorSimBridgeModule::ShutdownModule()
{
    // If this module is unloaded in the middle of simulation for some reason,
    // raise an error because we do not support this and we want to know when
    // this happens
    check(!ActorSpawnedDelegateHandle.IsValid());

    // remove event handlers used by this module
    FWorldDelegates::OnWorldCleanup.Remove(WorldCleanupDelegateHandle);
    WorldCleanupDelegateHandle.Reset();

    FWorldDelegates::OnPostWorldInitialization.Remove(PostWorldInitializationDelegateHandle);
    PostWorldInitializationDelegateHandle.Reset();
}

void FInteriorSimBridgeModule::PostWorldInitializationEventHandler(
    UWorld *World, const UWorld::InitializationValues)
{
    check(World);

    if (World->IsGameWorld())
    {
        // Required to handle cases when new actors of custom classes are spawned
        ActorSpawnedDelegateHandle = World->AddOnActorSpawnedHandler(FOnActorSpawned::FDelegate::CreateRaw(this, &FInteriorSimBridgeModule::ActorSpawnedEventHandler));
    }
}

void FInteriorSimBridgeModule::WorldCleanupEventHandler(UWorld *World,
                                                        bool bSessionEnded,
                                                        bool bCleanupResources)
{
    check(World);

    if (World->IsGameWorld())
    {
        // Remove event handlers bound to this world before world gets cleaned up
        World->RemoveOnActorSpawnedHandler(ActorSpawnedDelegateHandle);
        ActorSpawnedDelegateHandle.Reset();
    }
}

void FInteriorSimBridgeModule::ActorSpawnedEventHandler(AActor *InActor)
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

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FInteriorSimBridgeModule, InteriorSimBridge)
