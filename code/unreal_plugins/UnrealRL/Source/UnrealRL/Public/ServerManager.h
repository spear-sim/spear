#pragma once

#include "CoreMinimal.h"
#include "ServerManager.generated.h"

namespace unrealrl
{
class UnrealRLManager;
}

class UWorld;

/**
 * This class manages UnrealRLManager's lifecycle and connects UE simulation
 * with UnrealRLManager.
 */
UCLASS()
class UNREALRL_API UServerManager : public UObject
{
    GENERATED_BODY()

public:
    UServerManager(const FObjectInitializer& ObjectInitializer =
                       FObjectInitializer::Get());

    /** UObject overrides*/
    virtual UWorld* GetWorld() const override
    {
        return WorldInstance;
    }

    /** Utility functions*/
    FORCEINLINE static UServerManager& Get()
    {
        check(ServerManagerInstance);
        return *ServerManagerInstance;
    }

    FORCEINLINE static FRandomStream& GetRandomStream()
    {
        return RandomStream;
    }

    FORCEINLINE static void SetRandomStreamSeed(int32 InSeed)
    {
        RandomStream.Initialize(InSeed);
    }

    FORCEINLINE static int32 GetCurrentSeed()
    {
        return RandomStream.GetCurrentSeed();
    }

    FORCEINLINE static bool IsServerManagerReady()
    {
        return (ServerManagerInstance != nullptr);
    }

    FORCEINLINE static bool IsEnvironmentReady()
    {
        return bIsEnvironmentReady;
    }

    FORCEINLINE static void SetEnvironmentReady(bool bIsReady)
    {
        bIsEnvironmentReady = bIsReady;
    }

    virtual void ValidateEnvReadiness();

    /**map of actors names and its references captured during actor spawn*/
    TMap<FString, AActor*> ActorNameToRefMap;

    /**Contains Specific references to RL agents in the environment.*/
    UPROPERTY()
    TArray<AActor*> AgentActors;

    /** Delegate handlers*/
    FDelegateHandle ActorSpawnedDelegateHandle;

    FDelegateHandle WorldBeginPlayDelegateHandle;

    /** Called when world begins play.
     * Rpc server is created here.
     */
    void OnWorldBeginPlay();

    /** We need this as some actor are spawned later and we need to observe
     * them. */
    void OnActorSpawned(AActor* InActor);

    /** Delegates used */
    virtual void OnPostWorldInit(UWorld* World,
                                 const UWorld::InitializationValues);

    virtual void
    OnWorldCleanup(UWorld* World, bool bSessionEnded, bool bCleanupResources);

    // virtual void OnWorldBeginTearDown(UWorld* World);

protected:
    virtual void InitializeServer();

    virtual void SetCVarDefaultValues();

    virtual void ReadCommandLineValues();

    virtual void BindToDelegates();

    /** UObject overrides*/
    virtual void PostInitProperties() override;
    virtual void BeginDestroy() override;

private:
    /** Instance of this class. */
    static UServerManager* ServerManagerInstance;

    /** Random generator for custom seeding. */
    static FRandomStream RandomStream;

    /** RL manager that controls the game in an RL fashion. */
    unrealrl::UnrealRLManager* RLManager;

    UPROPERTY()
    UWorld* WorldInstance;

    UPROPERTY()
    int Port;

    UPROPERTY()
    FString Hostname;

    UPROPERTY()
    int32 Seed;

    UPROPERTY()
    bool bRLMode = true;

    static bool bIsEnvironmentReady;
};
