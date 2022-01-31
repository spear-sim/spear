// Copyright Epic Games, Inc. All Rights Reserved.

#include "InteriorSimBridge.h"

#include "EngineUtils.h"
#include "Engine/Engine.h"
#include "Engine/World.h"

#include "UrdfBot/SimModeUrdfBot.h"
#include "SimpleVehicle/SimModeSimpleVehicle.h"
#include "UrdfBotBrain.h"
#include "SimpleVehicleBrain.h"

#define LOCTEXT_NAMESPACE "FInteriorSimBridgeModule"

DEFINE_LOG_CATEGORY(LogInteriorSimBridge);

void FInteriorSimBridgeModule::StartupModule()
{
    // This code will execute after your module is loaded into memory; the exact
    // timing is specified in the .uplugin file per-module
    UE_LOG(LogInteriorSimBridge, Log, TEXT("InteriorSimBridge module loaded."));

    // required to get updated gameworld instance and add OnActorSpawned event
    // handler
    FWorldDelegates::OnPostWorldInitialization.AddRaw(
        this, &FInteriorSimBridgeModule::PostWorldInitializationEventHandler);

    // required to reset any custom logic during a world cleanup
    FWorldDelegates::OnWorldCleanup.AddRaw(
        this, &FInteriorSimBridgeModule::WorldCleanupEventHandler);

    // required to handle custom logic when actors are initialized
    FWorldDelegates::OnWorldInitializedActors.AddRaw(
        this, &FInteriorSimBridgeModule::WorldInitializedActorsEventHandler);
}

void FInteriorSimBridgeModule::PostWorldInitializationEventHandler(
    UWorld* World, const UWorld::InitializationValues)
{
    if (World && World->IsGameWorld())
    {
        UE_LOG(LogInteriorSimBridge, Log,
               TEXT("InteriorSimBridge: New world assigned. World "
                    "changed from %s "
                    "to %s"),
               *GetNameSafe(WorldInstance), *GetNameSafe(World));

        WorldInstance = World;

        UE_LOG(LogInteriorSimBridge, Log,
               TEXT("InteriorSimBridge: Binding OnActorSpawned "
                    "Delegate..."));

        // required to handle cases when new actors of custom classes are
        // spawned
        WorldInstance->AddOnActorSpawnedHandler(
            FOnActorSpawned::FDelegate::CreateRaw(
                this, &FInteriorSimBridgeModule::ActorSpawnedEventHandler));
    }
}

void FInteriorSimBridgeModule::WorldCleanupEventHandler(UWorld* World,
                                                        bool bSessionEnded,
                                                        bool bCleanupResources)
{
    UE_LOG(LogInteriorSimBridge, Log, TEXT("World %s is cleaning up"),
           *GetNameSafe(World));

    if (World && World->IsGameWorld())
    {
        if (World == WorldInstance)
        {
            WorldInstance = nullptr;
        }
    }
}

void FInteriorSimBridgeModule::WorldInitializedActorsEventHandler(
    const UWorld::FActorsInitializedParams& ActorsInitializedParams)
{
    // add required components to actors if not already present when initialized
    if (WorldInstance)
    {
        for (TActorIterator<AActor> It(WorldInstance, AActor::StaticClass());
             It; ++It)
        {
            // openbot
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
            // locobot
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
}

void FInteriorSimBridgeModule::ActorSpawnedEventHandler(AActor* InActor)
{
    UE_LOG(LogInteriorSimBridge, Log, TEXT("Spawned an Actor with name %s."),
           *(InActor->GetName()));

    // check UrdfBot factory and create corresponding UBrain component
    if (InActor->IsA(ASimModeUrdfBot::StaticClass()))
    {
        UE_LOG(LogInteriorSimBridge, Log,
               TEXT("Spawned actor has a UUrdfBotBrain component."));

        UUrdfBotBrain* Brain = NewObject<UUrdfBotBrain>(
            InActor, UUrdfBotBrain::StaticClass(), FName("UUrdfBotBrain"));

        Brain->RegisterComponent();
    }
    else if (InActor->IsA(ASimModeSimpleVehicle::StaticClass()))
    {
        UE_LOG(LogInteriorSimBridge, Log,
               TEXT("Spawned actor has a USimpleVehicleBrain component"));

        USimpleVehicleBrain* Brain = NewObject<USimpleVehicleBrain>(
            InActor, USimpleVehicleBrain::StaticClass(),
            FName("USimpleVehicleBrain"));

        Brain->RegisterComponent();
    }
}

void FInteriorSimBridgeModule::ShutdownModule()
{
    // This function may be called during shutdown to clean up your module.  For
    // modules that support dynamic reloading, we call this function before
    // unloading the module.
    UE_LOG(LogInteriorSimBridge, Log,
           TEXT("InteriorSimBridge module unloaded."));

    WorldInstance = nullptr;
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FInteriorSimBridgeModule, InteriorSimBridge)
