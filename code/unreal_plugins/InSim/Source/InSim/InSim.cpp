// Copyright Epic Games, Inc. All Rights Reserved.

#include "InSim.h"
#include "InSimManager.h"

#include "EngineUtils.h"
#include "Engine/Engine.h"

#define LOCTEXT_NAMESPACE "FInSimModule"

DEFINE_LOG_CATEGORY(LogInSIM);

struct FInSimManagerBootLoader
{
    FInSimManagerBootLoader()
    {
        FCoreDelegates::OnPostEngineInit.AddLambda(
            [this]() { OnEngineLoopInitComplete(); });
    }

    void OnEngineLoopInitComplete()
    {
        UClass* Class = UInSimManager::StaticClass();

        UInSimManager* InSimManagerInstance =
            NewObject<UInSimManager>(GEngine, Class);
        check(InSimManagerInstance);
        InSimManagerInstance->AddToRoot();
        UE_LOG(LogInSIM, Warning, TEXT("Created an instance of SimManager!!"));
    }
};

void FInSimModule::StartupModule()
{
    // This code will execute after your module is loaded into memory; the exact
    // timing is specified in the .uplugin file per-module
    UE_LOG(LogInSIM, Log, TEXT("InSim module loaded."));

    FInSimManagerBootLoader Loader;
}

void FInSimModule::ShutdownModule()
{
    // This function may be called during shutdown to clean up your module.  For
    // modules that support dynamic reloading, we call this function before
    // unloading the module.
    UE_LOG(LogInSIM, Log, TEXT("InSim module unloaded."));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FInSimModule, InSim)
