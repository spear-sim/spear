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
    // required to get updated gameworld instance and add OnActorSpawned event
    // handler
    PostWorldInitializationDelegateHandle =
        FWorldDelegates::OnPostWorldInitialization.AddRaw(
            this,
            &FInteriorSimBridgeModule::PostWorldInitializationEventHandler);

    // required to reset any custom logic during a world cleanup
    WorldCleanupDelegateHandle = FWorldDelegates::OnWorldCleanup.AddRaw(
        this, &FInteriorSimBridgeModule::WorldCleanupEventHandler);

    // required to handle custom logic when actors are initialized
    WorldInitializedActorsDelegateHandle =
        FWorldDelegates::OnWorldInitializedActors.AddRaw(
            this,
            &FInteriorSimBridgeModule::WorldInitializedActorsEventHandler);
}

void FInteriorSimBridgeModule::PostWorldInitializationEventHandler(
    UWorld* InWorld, const UWorld::InitializationValues)
{
    check(InWorld);

    if (InWorld->IsGameWorld())
    {
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
        check(World == InWorld);

        // remove event handlers bound to this world before world gets cleaned
        // up
        World->RemoveOnActorSpawnedHandler(ActorSpawnedDelegateHandle);
        ActorSpawnedDelegateHandle.Reset();

        // clear local reference to world as it will get cleaned up soon
        World = nullptr;
    }
}

void FInteriorSimBridgeModule::WorldInitializedActorsEventHandler(
    const UWorld::FActorsInitializedParams& ActorsInitializedParams)
{
    check(World);

    // add required components to actors if not already present when initialized
    for (TActorIterator<AActor> It(World, AActor::StaticClass()); It; ++It)
    {
        // openbot, and other simplevehicle based robots
        if ((*It)->GetName().Contains(TEXT("simmodesimplevehicle"),
                                      ESearchCase::IgnoreCase) &&
            !(*It)->GetName().Contains(TEXT("simmode"),
                                       ESearchCase::IgnoreCase))
        {
            if (!(*It)->GetComponentByClass(UBrain::StaticClass()))
            {
                USimpleVehicleBrain* Brain = NewObject<USimpleVehicleBrain>(
                    (*It), USimpleVehicleBrain::StaticClass(),
                    FName("USimpleVehicleBrain"));

                Brain->RegisterComponent();
            }
        }
        // locobot, and other UrdfBot based robots
        else if ((*It)->GetName().Contains(TEXT("simmodeurdfbot"),
                                           ESearchCase::IgnoreCase))
        {
            if (!(*It)->GetComponentByClass(UBrain::StaticClass()))
            {
                UUrdfBotBrain* Brain = NewObject<UUrdfBotBrain>(
                    (*It), UUrdfBotBrain::StaticClass(),
                    FName("UUrdfBotBrain"));

                Brain->RegisterComponent();
            }
        }
    }
}

void FInteriorSimBridgeModule::ActorSpawnedEventHandler(AActor* InActor)
{
    check(InActor);

    // check UrdfBot factory and create corresponding UBrain component
    if (InActor->IsA(ASimModeUrdfBot::StaticClass()))
    {
        UUrdfBotBrain* Brain = NewObject<UUrdfBotBrain>(
            InActor, UUrdfBotBrain::StaticClass(), FName("UUrdfBotBrain"));

        check(Brain);
        Brain->RegisterComponent();
    }
    else if (InActor->IsA(ASimModeSimpleVehicle::StaticClass()))
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

    FWorldDelegates::OnWorldInitializedActors.Remove(
        WorldInitializedActorsDelegateHandle);
    WorldInitializedActorsDelegateHandle.Reset();

    World = nullptr;
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FInteriorSimBridgeModule, InteriorSimBridge)
