// Copyright Epic Games, Inc. All Rights Reserved.

#include "InteriorSimBridge.h"
#include "InteriorSimBridgeManager.h"

#include "EngineUtils.h"
#include "Engine/Engine.h"

#define LOCTEXT_NAMESPACE "FInteriorSimBridgeModule"

DEFINE_LOG_CATEGORY(LogInteriorSimBridge);

struct FInteriorSimBridgeManagerBootLoader
{
    FInteriorSimBridgeManagerBootLoader()
    {
        FCoreDelegates::OnPostEngineInit.AddLambda(
            [this]() { OnEngineLoopInitComplete(); });
    }

    void OnEngineLoopInitComplete()
    {
        UClass* Class = UInteriorSimBridgeManager::StaticClass();

        UInteriorSimBridgeManager* InteriorSimBridgeManagerInstance =
            NewObject<UInteriorSimBridgeManager>(GEngine, Class);
        check(InteriorSimBridgeManagerInstance);
        InteriorSimBridgeManagerInstance->AddToRoot();
        UE_LOG(LogInteriorSimBridge, Warning, TEXT("Created an instance of SimManager!!"));
    }
};

void FInteriorSimBridgeModule::StartupModule()
{
    // This code will execute after your module is loaded into memory; the exact
    // timing is specified in the .uplugin file per-module
    UE_LOG(LogInteriorSimBridge, Log, TEXT("InteriorSimBridge module loaded."));

    FInteriorSimBridgeManagerBootLoader Loader;
}

void FInteriorSimBridgeModule::ShutdownModule()
{
    // This function may be called during shutdown to clean up your module.  For
    // modules that support dynamic reloading, we call this function before
    // unloading the module.
    UE_LOG(LogInteriorSimBridge, Log, TEXT("InteriorSimBridge module unloaded."));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FInteriorSimBridgeModule, InteriorSimBridge)
