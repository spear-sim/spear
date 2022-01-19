// Copyright Epic Games, Inc. All Rights Reserved.

#include "UnrealRL.h"
#include "ServerManager.h"

#define LOCTEXT_NAMESPACE "FUnrealRLModule"

DEFINE_LOG_CATEGORY(LogRL);

struct FServerManagerLoader
{
    FServerManagerLoader()
    {
        FCoreDelegates::OnPostEngineInit.AddLambda(
            [this]() { OnPostEngineInit(); });
    }

    void OnPostEngineInit()
    {
        UClass* Class = UServerManager::StaticClass();
        UE_LOG(LogRL, Warning, TEXT("Creating ServerManager of class %s"),
               *GetNameSafe(Class));

        UServerManager* ServerManagerInstance =
            NewObject<UServerManager>(GEngine, Class);
        check(ServerManagerInstance);
        ServerManagerInstance
            ->AddToRoot(); // To prevent from being garbage collected
        // TODO: Would it be possible to - instead of AddToRoot - use a private
        // member in UnrealRL.h?
    }
};

void FUnrealRLModule::StartupModule()
{
    // This code will execute after your module is loaded into memory; the exact
    // timing is specified in the .uplugin file per-module.
    UE_LOG(LogRL, Log, TEXT("UnrealRL module loaded."));

    // Create ServerManagerLoader, for lifetime of the UnrealRL module.
    FServerManagerLoader Loader;
}

void FUnrealRLModule::ShutdownModule()
{
    // This function may be called during shutdown to clean up your module.  For
    // modules that support dynamic reloading, we call this function before
    // unloading the module.
    UE_LOG(LogRL, Log, TEXT("UnrealRL module unloaded."));

    // TODO: Do we need to free the ServerManagerInstance?
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FUnrealRLModule, UnrealRL)
