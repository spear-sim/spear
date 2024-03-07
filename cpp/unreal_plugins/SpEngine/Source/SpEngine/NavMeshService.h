//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts>  // std::same_as

#include <SpEngine/EngineService.h> // CEntryPointBinder

class NavMeshService {
public:
	NavMeshService() = delete;
	NavMeshService(CEntryPointBinder auto* entry_point_binder)
	{
		//entry_point_binder->bind(...);
	}
};
#pragma once
