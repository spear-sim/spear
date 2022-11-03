#include "SceneManager.h"

#include "PhysicsManager.h"

void SceneManager::StartupModule()
{
    // TODO: need a config flag to enable/disable, because SceneManager won't work when a non-Kujiale scene is loaded
    // PhysicsManager::initialize();
}

void SceneManager::ShutdownModule()
{
    // TODO: see above
    // PhysicsManager::terminate();
}

IMPLEMENT_MODULE(SceneManager, SceneManager)
