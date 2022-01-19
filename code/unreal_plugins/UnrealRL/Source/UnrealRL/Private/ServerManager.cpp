#include "ServerManager.h"

#include "EngineUtils.h"
#include "Engine/Engine.h"
#include "GameFramework/GameModeBase.h"
#include "UnrealRL.h"
#include "UnrealRLManager.h"
#include "RL/Brain.h"

#include <thread>

// Static variables definitions
FRandomStream UServerManager::RandomStream = FRandomStream(FMath::Rand());
UServerManager* UServerManager::ServerManagerInstance = nullptr;
bool UServerManager::bIsEnvironmentReady = false;

UServerManager::UServerManager(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    if (HasAnyFlags(RF_ClassDefaultObject) == false)
    {
        ensure(ServerManagerInstance == nullptr);
        ServerManagerInstance = this;
        UE_LOG(LogRL, Warning,
               TEXT("ServerManagerInstance successfully assigned..."));
    }

    Hostname = TEXT("localhost");
    Port = 8080u;
}

void UServerManager::PostInitProperties()
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

void UServerManager::BindToDelegates()
{
    FWorldDelegates::OnPostWorldInitialization.AddUObject(
        this, &UServerManager::OnPostWorldInit);

    FWorldDelegates::OnWorldCleanup.AddUObject(this,
                                               &UServerManager::OnWorldCleanup);

    // FWorldDelegates::OnWorldBeginTearDown.AddUObject(this,
    // &UServerManager::OnWorldBeginTearDown);
}

void UServerManager::OnPostWorldInit(UWorld* World,
                                     const UWorld::InitializationValues)
{
    UE_LOG(LogRL, Log, TEXT("OnPostWorldInit called on new world %s"),
           *GetNameSafe(World));
    if (World && World->IsGameWorld())
    {
        UE_LOG(LogRL, Log,
               TEXT("New world assigned. World changed from %s to %s"),
               *GetNameSafe(WorldInstance), *GetNameSafe(World));
        WorldInstance = World;

        if (ActorSpawnedDelegateHandle.IsValid() == false)
        {
            UE_LOG(LogRL, Log, TEXT("Binding OnActorSpawned Delegate..."));
            ActorSpawnedDelegateHandle =
                WorldInstance->AddOnActorSpawnedHandler(
                    FOnActorSpawned::FDelegate::CreateUObject(
                        this, &UServerManager::OnActorSpawned));
        }

        if (WorldBeginPlayDelegateHandle.IsValid() == false)
        {
            UE_LOG(LogRL, Log, TEXT("Binding OnWorldBeginPlay Delegate..."));
            WorldBeginPlayDelegateHandle =
                WorldInstance->OnWorldBeginPlay.AddUObject(
                    this, &UServerManager::OnWorldBeginPlay);
        }
    }
}

void UServerManager::ReadCommandLineValues()
{
    if (FParse::Value(FCommandLine::Get(), TEXT("rlseed="), Seed))
    {
        UServerManager::SetRandomStreamSeed(Seed);
    }
    UE_LOG(LogRL, Log, TEXT("Using Random Generator with seed = %d"),
           UServerManager::GetCurrentSeed());

    FParse::Value(FCommandLine::Get(), TEXT("rlport="), Port);
    FParse::Value(FCommandLine::Get(), TEXT("rlip="), Hostname);
    UE_LOG(LogRL, Log, TEXT("Server is listening on ip=%s, and port=%d"),
           *Hostname, Port);
}

void UServerManager::InitializeServer()
{
    // Start rpc server
    if (!RLManager)
    {
        RLManager = new unrealrl::UnrealRLManager(
            TCHAR_TO_UTF8(*Hostname),
            Port); // TCHAR_TO_UTF8 for FString -> std::string
    }

    if (RLManager)
    {
        RLManager->AsyncRun(std::max(std::thread::hardware_concurrency(), 4u) -
                            2u);
        UE_LOG(LogRL, Log, TEXT("Successfully started rpc server!"));
    }
    else
    {
        UE_LOG(LogRL, Error,
               TEXT("Could not create an UnrealRLManager. RL experiment cannot "
                    "be performed."));
    }
}

void UServerManager::SetCVarDefaultValues()
{
    UE_LOG(LogRL, Log, TEXT("Setting default CVar values..."));
    // Set few console commands for syncing GT and draw threads
    // GTSyncType -
    // http://docs.unrealengine.com/en-US/SharingAndReleasing/LowLatencyFrameSyncing/index.html
    // We want GT to sync to RHI thread
    GEngine->Exec(GetWorld(), TEXT("r.GTSyncType 1"));
    GEngine->Exec(GetWorld(), TEXT("r.OneFrameThreadLag 0"));
}

void UServerManager::OnWorldBeginPlay()
{
    UE_LOG(LogRL, Log, TEXT("World %s has begun to play"),
           *GetNameSafe(WorldInstance));

    // Read Command Line values
    ReadCommandLineValues();

    // create and start rpc server
    InitializeServer();

    // default CVar
    SetCVarDefaultValues();

    // Make a map
    for (TActorIterator<AActor> It(WorldInstance, AActor::StaticClass()); It;
         ++It)
    {
        if (ActorNameToRefMap.Contains((*It)->GetName()) == false)
        {
            ActorNameToRefMap.Emplace(AActor::GetDebugName(*It), *It);

            // Capture Actors which have UBrain as a component
            if ((*It)->GetComponentByClass(UBrain::StaticClass()))
            {
                AgentActors.Emplace(*It);
                UE_LOG(LogRL, Log, TEXT("Found an RL agent %s"),
                       *((*It)->GetName()));
            }
        }
    }

    // RLMode == Synchronous Mode
    if (bRLMode)
    {
        RLManager->SetSynchronousMode(true);
        UE_LOG(LogRL, Log,
               TEXT("Setting synchronous mode for this experiment..."));
    }
    else
    {
        RLManager->SetSynchronousMode(false);
        UE_LOG(LogRL, Log,
               TEXT("Setting Asynchronous mode for this experiment..."));
    }
}

void UServerManager::OnWorldCleanup(UWorld* World,
                                    bool bSessionEnded,
                                    bool bCleanupResources)
{
    UE_LOG(LogRL, Log, TEXT("World %s is cleaning up..."), *GetNameSafe(World));

    // no need to remove, the World is going away
    if (World && World->IsGameWorld())
    {
        if (World == WorldInstance)
        {
            ActorNameToRefMap.Reset();
            AgentActors.Reset();

            if (ActorSpawnedDelegateHandle.IsValid() == true)
            {
                WorldInstance->RemoveOnActorSpawnedHandler(
                    ActorSpawnedDelegateHandle);
                ActorSpawnedDelegateHandle.Reset();
            }

            if (WorldBeginPlayDelegateHandle.IsValid() == true)
            {
                WorldInstance->OnWorldBeginPlay.Remove(
                    WorldBeginPlayDelegateHandle);
                WorldBeginPlayDelegateHandle.Reset();
            }

            WorldInstance = nullptr;

            if (RLManager)
            {
                RLManager->Stop();
                delete RLManager;
                RLManager = nullptr;
                UE_LOG(LogRL, Log,
                       TEXT("UServerManager::OnWorldCleanUp() : Closing the "
                            "RPC server..."));
            }
        }
    }
}

void UServerManager::OnActorSpawned(AActor* InActor)
{
    // check if we do not already have this actor
    if (ActorNameToRefMap.Contains(InActor->GetName()) == false)
    {
        ActorNameToRefMap.Emplace(InActor->GetName(), InActor);

        // Capture Actors which have UBrain as a component
        if (InActor->GetComponentByClass(UBrain::StaticClass()))
        {
            AgentActors.Emplace(InActor);
            UE_LOG(LogRL, Log, TEXT("Found an RL agent %s"),
                   *InActor->GetName());
        }
    }
}

void UServerManager::ValidateEnvReadiness()
{
    uint8 ReadyCount = 0;

    for (TActorIterator<AActor> It(WorldInstance, AActor::StaticClass()); It;
         ++It)
    {
        // Capture Actors which have UBrain as a component
        if ((*It)->GetComponentByClass(UBrain::StaticClass()))
        {
            if (ActorNameToRefMap.Contains((*It)->GetName()) == false)
            {
                ActorNameToRefMap.Emplace((*It)->GetName(), (*It));
            }

            if (!AgentActors.FindByKey((*It)))
            {
                AgentActors.Emplace((*It));
            }

            UBrain* BrainComponent =
                Cast<UBrain>((*It)->GetComponentByClass(UBrain::StaticClass()));
            if (BrainComponent && BrainComponent->IsAgentReady())
            {
                ReadyCount++;
            }
        }
    }

    // If all rl agents are ready, env is ready
    if (ReadyCount == AgentActors.Num())
    {
        SetEnvironmentReady(true);
    }
    else
    {
        SetEnvironmentReady(false);
    }
}

void UServerManager::BeginDestroy()
{
    if (RLManager)
    {
        RLManager->Stop();
        delete RLManager;
        RLManager = nullptr;
        UE_LOG(
            LogRL, Log,
            TEXT("UServerManager::BeginDestroy() : Closing the RPC server..."));
    }

    if (ServerManagerInstance == this)
    {
        ServerManagerInstance = nullptr;
        UE_LOG(LogRL, Log,
               TEXT("Resetting ServerManagerInstance to nullptr..."));
    }

    if (ActorSpawnedDelegateHandle.IsValid() == true)
    {
        WorldInstance->RemoveOnActorSpawnedHandler(ActorSpawnedDelegateHandle);
        ActorSpawnedDelegateHandle.Reset();
    }

    if (WorldBeginPlayDelegateHandle.IsValid() == true)
    {
        WorldInstance->OnWorldBeginPlay.Remove(WorldBeginPlayDelegateHandle);
        WorldBeginPlayDelegateHandle.Reset();
    }

    Super::BeginDestroy();
}
