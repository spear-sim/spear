#include "InteriorSimBridgeManager.h"
#include "RL/Brain.h"
#include "UrdfBotBrain.h"
#include "UrdfBot/SimModeUrdfBot.h"
#include "SimpleVehicleBrain.h"
#include "SimpleVehicle/SimModeSimpleVehicle.h"
#include "InteriorSimBridge.h"

#include "EngineUtils.h"
#include "Engine/Engine.h"

#include <thread>

// Static variables definitions
UInteriorSimBridgeManager*
    UInteriorSimBridgeManager::InteriorSimBridgeManagerInstance = nullptr;

UInteriorSimBridgeManager::UInteriorSimBridgeManager(
    const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    if (HasAnyFlags(RF_ClassDefaultObject) == false)
    {
        ensure(InteriorSimBridgeManagerInstance == nullptr);
        InteriorSimBridgeManagerInstance = this;
        UE_LOG(
            LogInteriorSimBridge, Warning,
            TEXT("InteriorSimBridgeManagerInstance successfully assigned..."));
    }
}

void UInteriorSimBridgeManager::PostInitProperties()
{
    Super::PostInitProperties();

    if (HasAnyFlags(RF_ClassDefaultObject) == false)
    {
        BindToDelegates();

        // if there's any world present
        UWorld* World =
#if WITH_EDITOR
            GIsEditor ? GWorld :
#endif // WITH_EDITOR
                      (GEngine->GetWorldContexts().Num() > 0
                           ? GEngine->GetWorldContexts()[0].World()
                           : nullptr);

        OnPostWorldInit(World, UWorld::InitializationValues());
    }
}

void UInteriorSimBridgeManager::BindToDelegates()
{
    // required to obtained gameworld instance
    FWorldDelegates::OnPostWorldInitialization.AddUObject(
        this, &UInteriorSimBridgeManager::OnPostWorldInit);

    // required to reset any delegates while clearing world
    FWorldDelegates::OnWorldCleanup.AddUObject(
        this, &UInteriorSimBridgeManager::OnWorldCleanup);

    // to catch any new actors spawned with Brain component
    FWorldDelegates::OnWorldInitializedActors.AddUObject(
        this, &UInteriorSimBridgeManager::OnWorldInitializedActors);
}

void UInteriorSimBridgeManager::OnPostWorldInit(
    UWorld* World, const UWorld::InitializationValues)
{
    // UE_LOG(LogInteriorSimBridge, Warning, TEXT("OnPostWorldInit called on new
    // world %s"), *GetNameSafe(World));

    if (World && World->IsGameWorld())
    {
        UE_LOG(LogInteriorSimBridge, Warning,
               TEXT("InteriorSimBridgeManager: New world assigned. World "
                    "changed from %s "
                    "to %s"),
               *GetNameSafe(WorldInstance), *GetNameSafe(World));

        WorldInstance = World;

        if (ActorSpawnedDelegateHandle.IsValid() == false)
        {
            UE_LOG(LogInteriorSimBridge, Warning,
                   TEXT("InteriorSimBridgeManager: Binding OnActorSpawned "
                        "Delegate..."));
            ActorSpawnedDelegateHandle =
                WorldInstance->AddOnActorSpawnedHandler(
                    FOnActorSpawned::FDelegate::CreateUObject(
                        this, &UInteriorSimBridgeManager::OnActorSpawned));
        }
    }
}

void UInteriorSimBridgeManager::OnWorldCleanup(UWorld* World,
                                               bool bSessionEnded,
                                               bool bCleanupResources)
{
    UE_LOG(LogInteriorSimBridge, Warning,
           TEXT("InteriorSimBridgeManager: : World %s is cleaning up..."),
           *GetNameSafe(World));

    // no need to remove, the World is going away
    if (World && World->IsGameWorld())
    {
        if (World == WorldInstance)
        {

            if (ActorSpawnedDelegateHandle.IsValid() == true)
            {
                WorldInstance->RemoveOnActorSpawnedHandler(
                    ActorSpawnedDelegateHandle);
                ActorSpawnedDelegateHandle.Reset();
            }

            WorldInstance = nullptr;
        }
    }
}

void UInteriorSimBridgeManager::OnWorldInitializedActors(
    const UWorld::FActorsInitializedParams& ActorsInitializedParams)
{
    UE_LOG(LogInteriorSimBridge, Warning,
           TEXT("InteriorSimBridgeManager : OnWorldInitialized Actors..."));

    // Find a URDF robot
    if (WorldInstance)
    {
        for (TActorIterator<AActor> It(WorldInstance, AActor::StaticClass());
             It; ++It)
        {
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

void UInteriorSimBridgeManager::OnActorSpawned(AActor* InActor)
{
    UE_LOG(LogInteriorSimBridge, Warning,
           TEXT("InteriorSimBridgeManager : OnActorSpawned... %s"),
           *(InActor->GetName()));

    // Capture UrdfBot factory and spawn corresponding UBrain
    // identify ASimModeUrdfBot by actor label
    if (InActor->IsA(ASimModeUrdfBot::StaticClass()))
    {
        UE_LOG(
            LogInteriorSimBridge, Warning,
            TEXT("InteriorSimBridgeManager : OnActorSpawned... UUrdfBotBrain"));
        UUrdfBotBrain* Brain = NewObject<UUrdfBotBrain>(
            InActor, UUrdfBotBrain::StaticClass(), FName("UUrdfBotBrain"));
        Brain->RegisterComponent();
    }

    if (InActor->IsA(ASimModeSimpleVehicle::StaticClass()))
    {
        UE_LOG(LogInteriorSimBridge, Warning,
               TEXT("InteriorSimBridgeManager : OnActorSpawned... "
                    "USimpleVehicleBrain"));
        USimpleVehicleBrain* Brain = NewObject<USimpleVehicleBrain>(
            InActor, USimpleVehicleBrain::StaticClass(),
            FName("USimpleVehicleBrain"));
        Brain->RegisterComponent();
    }
}

void UInteriorSimBridgeManager::BeginDestroy()
{
    if (InteriorSimBridgeManagerInstance == this)
    {
        InteriorSimBridgeManagerInstance = nullptr;
        UE_LOG(LogInteriorSimBridge, Warning,
               TEXT("InteriorSimBridgeManager: Resetting "
                    "InteriorSimBridgeManagerInstance to nullptr..."));
    }

    if (ActorSpawnedDelegateHandle.IsValid() == true)
    {
        WorldInstance->RemoveOnActorSpawnedHandler(ActorSpawnedDelegateHandle);
        ActorSpawnedDelegateHandle.Reset();
    }

    Super::BeginDestroy();
}
