//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts>

template <typename TEntryPointBinder>
concept CEntryPointBinder = requires(TEntryPointBinder entry_point_binder){
	{ entry_point_binder.bind("", "", []() -> void {}) } -> std::same_as<void>;
};

class GameWorldService {
public:
	GameWorldService() = default;
	GameWorldService(CEntryPointBinder auto* entry_point_binder)
	{
		//entry_point_binder_->bind(...);
	}
};
