#include "InSimManager.h"
#include "RL/Brain.h"
#include "UrdfBotBrain.h"
#include "UrdfBot/SimModeUrdfBot.h"
#include "SimpleVehicleBrain.h"
#include "SimpleVehicle/SimModeSimpleVehicle.h"
#include "InSim.h"

#include "EngineUtils.h"
#include "Engine/Engine.h"

#include <thread>

// Static variables definitions
UInSimManager* UInSimManager::InSimManagerInstance = nullptr;

UInSimManager::UInSimManager(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    if (HasAnyFlags(RF_ClassDefaultObject) == false)
    {
        ensure(InSimManagerInstance == nullptr);
        InSimManagerInstance = this;
        UE_LOG(LogInSIM, Warning,
               TEXT("InSimManagerInstance successfully assigned..."));
    }
}

void UInSimManager::PostInitProperties()
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

void UInSimManager::BindToDelegates()
{
    // required to obtained gameworld instance
    FWorldDelegates::OnPostWorldInitialization.AddUObject(
        this, &UInSimManager::OnPostWorldInit);

    // required to reset any delegates while clearing world
    FWorldDelegates::OnWorldCleanup.AddUObject(this,
                                               &UInSimManager::OnWorldCleanup);

    // to catch any new actors spawned with Brain component
    FWorldDelegates::OnWorldInitializedActors.AddUObject(
        this, &UInSimManager::OnWorldInitializedActors);
}

void UInSimManager::OnPostWorldInit(UWorld* World,
                                    const UWorld::InitializationValues)
{
    // UE_LOG(LogInSIM, Warning, TEXT("OnPostWorldInit called on new world %s"),
    // *GetNameSafe(World));

    if (World && World->IsGameWorld())
    {
        UE_LOG(LogInSIM, Warning,
               TEXT("InSimManager: New world assigned. World changed from %s "
                    "to %s"),
               *GetNameSafe(WorldInstance), *GetNameSafe(World));

        WorldInstance = World;

        if (ActorSpawnedDelegateHandle.IsValid() == false)
        {
            UE_LOG(LogInSIM, Warning,
                   TEXT("InSimManager: Binding OnActorSpawned Delegate..."));
            ActorSpawnedDelegateHandle =
                WorldInstance->AddOnActorSpawnedHandler(
                    FOnActorSpawned::FDelegate::CreateUObject(
                        this, &UInSimManager::OnActorSpawned));
        }
    }
}

void UInSimManager::OnWorldCleanup(UWorld* World,
                                   bool bSessionEnded,
                                   bool bCleanupResources)
{
    UE_LOG(LogInSIM, Warning,
           TEXT("InSimManager: : World %s is cleaning up..."),
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

void UInSimManager::OnWorldInitializedActors(
    const UWorld::FActorsInitializedParams& ActorsInitializedParams)
{
    UE_LOG(LogInSIM, Warning,
           TEXT("InSIMManager : OnWorldInitialized Actors..."));

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

void UInSimManager::OnActorSpawned(AActor* InActor)
{
    UE_LOG(LogInSIM, Warning, TEXT("InSIMManager : OnActorSpawned... %s"),
           *(InActor->GetName()));

    // Capture UrdfBot factory and spawn corresponding UBrain
    // identify ASimModeUrdfBot by actor label
    if (InActor->IsA(ASimModeUrdfBot::StaticClass()))
    {
        UE_LOG(LogInSIM, Warning,
               TEXT("InSIMManager : OnActorSpawned... UUrdfBotBrain"));
        UUrdfBotBrain* Brain = NewObject<UUrdfBotBrain>(
            InActor, UUrdfBotBrain::StaticClass(), FName("UUrdfBotBrain"));
        Brain->RegisterComponent();
    }

    if (InActor->IsA(ASimModeSimpleVehicle::StaticClass()))
    {
        UE_LOG(LogInSIM, Warning,
               TEXT("InSIMManager : OnActorSpawned... USimpleVehicleBrain"));
        USimpleVehicleBrain* Brain = NewObject<USimpleVehicleBrain>(
            InActor, USimpleVehicleBrain::StaticClass(),
            FName("USimpleVehicleBrain"));
        Brain->RegisterComponent();
    }
}

void UInSimManager::BeginDestroy()
{
    if (InSimManagerInstance == this)
    {
        InSimManagerInstance = nullptr;
        UE_LOG(
            LogInSIM, Warning,
            TEXT("InSimManager: Resetting InSimManagerInstance to nullptr..."));
    }

    if (ActorSpawnedDelegateHandle.IsValid() == true)
    {
        WorldInstance->RemoveOnActorSpawnedHandler(ActorSpawnedDelegateHandle);
        ActorSpawnedDelegateHandle.Reset();
    }

    Super::BeginDestroy();
}
