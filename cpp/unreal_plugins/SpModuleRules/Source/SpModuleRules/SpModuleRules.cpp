//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

// use if module does not implement any Unreal classes
IMPLEMENT_MODULE(FDefaultModuleImpl, SpModuleRules);

// use if module implements any Unreal classes
// IMPLEMENT_GAME_MODULE(FDefaultGameModuleImpl, SpearSimEditor);
