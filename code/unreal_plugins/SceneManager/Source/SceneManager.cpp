#include "SceneManager.h"

#include "PhysicsManager.h"

void SceneManager::StartupModule()
{
    PhysicsManager::initialize();
}

void SceneManager::ShutdownModule()
{
    PhysicsManager::terminate();
}

IMPLEMENT_MODULE(SceneManager, SceneManager)