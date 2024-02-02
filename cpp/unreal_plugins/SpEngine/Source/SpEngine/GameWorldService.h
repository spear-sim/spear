//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include "SpEngine/EngineService.h"

template <typename T>
concept CEntryPointBinder = requires(T engine_service){
	engine_service.bind();
};

template<CEntryPointBinder TEntryPointBinder>
class GameWorldService {
	
	GameWorldService() == delete;
	GameWorldService(TEntryPointBinder* entry_point_binder)
	{
		entry_point_binder_ = entry_point_binder;

		//entry_point_binder_->bind(...);
	}

	TEntryPointBinder* entry_point_binder_ = nullptr;
};
