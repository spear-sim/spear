//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/World.h>

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

class WorldService : public Service {
public:
    WorldService() = delete;
    WorldService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        // Entry points for miscellaneous functions that are accessible via getWorld(), but are not
        // accessible via the property system.

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("world_service", "get_first_player_controller",
            [this]() -> uint64_t {
                return toUInt64(getWorld()->GetFirstPlayerController()); // GetFirstPlayerController() isn't a UFUNCTION
            });
    }

    ~WorldService() = default;
};
