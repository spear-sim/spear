#pragma once

#include "CoreMinimal.h"
#include "Engine/World.h"
#include "InSimManager.generated.h"

class UWorld;

UCLASS()
class INSIM_API UInSimManager : public UObject
{
    GENERATED_BODY()

public:
    UInSimManager(const FObjectInitializer& ObjectInitializer =
                      FObjectInitializer::Get());

    /** UObject overrides*/
    virtual UWorld* GetWorld() const override
    {
        return WorldInstance;
    }

    /** Utility functions*/
    FORCEINLINE static UInSimManager& Get()
    {
        check(InSimManagerInstance);
        return *InSimManagerInstance;
    }

    FORCEINLINE static bool IsInSimManagerReady()
    {
        return (InSimManagerInstance != nullptr);
    }

protected:
    virtual void BindToDelegates();

    /** UObject overrides*/
    virtual void PostInitProperties() override;
    virtual void BeginDestroy() override;

    /** Delegate callbacks*/
    virtual void OnPostWorldInit(UWorld* World,
                                 const UWorld::InitializationValues);

    virtual void
    OnWorldCleanup(UWorld* World, bool bSessionEnded, bool bCleanupResources);

    void OnWorldInitializedActors(
        const UWorld::FActorsInitializedParams& ActorsInitializedParams);

    /** We need this as some actor are spawned later and we need to observe
     * them. */
    void OnActorSpawned(AActor* InActor);

private:
    /** Instance of this class. */
    static UInSimManager* InSimManagerInstance;

    UPROPERTY()
    UWorld* WorldInstance;

    /** Delegate handlers*/
    FDelegateHandle ActorSpawnedDelegateHandle;
};
