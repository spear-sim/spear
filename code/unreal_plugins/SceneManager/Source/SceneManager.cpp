#include "SceneManager.h"

#include "PhysicsManager.h"

void FSceneManagerModule::StartupModule()
{
    PhysicsManager::initialize();
}

void FSceneManagerModule::ShutdownModule()
{
    PhysicsManager::terminate();
}

IMPLEMENT_MODULE(FSceneManagerModule, SceneManager)