//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <map>
#include <string>
#include <utility> // std::move
#include <vector>

#include <NavigationData.h>
#include <NavigationSystem.h>
#include <NavMesh/RecastNavMesh.h>
#include <Templates/Casts.h>

#include "SpCore/Assert.h"
#include "SpCore/SpArray.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/MsgpackAdaptors.h"
#include "SpServices/Service.h"
#include "SpServices/SharedMemoryService.h"

class NavigationService : public Service {
public:
    NavigationService() = delete;
    NavigationService(CUnrealEntryPointBinder auto* unreal_entry_point_binder, SharedMemoryService* shared_memory_service)
    {
        SP_ASSERT(unreal_entry_point_binder);
        SP_ASSERT(shared_memory_service);

        shared_memory_service_ = shared_memory_service;

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("navigation_service", "get_nav_data_for_agent_name", 
            [this](uint64_t& navigation_system, std::string& agent_name) -> uint64_t {
                UNavigationSystemV1* navigation_system_ptr = toPtr<UNavigationSystemV1>(navigation_system);
                SP_ASSERT(navigation_system_ptr);
                return toUInt64(navigation_system_ptr->GetNavDataForAgentName(Unreal::toFName(agent_name)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("navigation_service", "get_random_points",
            [this](uint64_t& navigation_data, int& num_points, SpPackedArray& out_array) -> SpPackedArray {

                ANavigationData* navigation_data_ptr = toPtr<ANavigationData>(navigation_data);
                SP_ASSERT(navigation_data_ptr);
                ARecastNavMesh* recast_nav_mesh = Cast<ARecastNavMesh>(navigation_data_ptr); // no RTTI available, so use Cast instead of dynamic_cast
                SP_ASSERT(recast_nav_mesh);

                std::vector<double> points;
                for (int i = 0; i < num_points; i++) {
                    FVector point = recast_nav_mesh->GetRandomPoint().Location;
                    points.push_back(point.X);
                    points.push_back(point.Y);
                    points.push_back(point.Z);
                }

                std::vector<int64_t> shape = {-1, 3};
                SpArraySharedMemoryUsageFlags usage_flags = SpArraySharedMemoryUsageFlags::Arg | SpArraySharedMemoryUsageFlags::ReturnValue;
                std::map<std::string, SpArraySharedMemoryView> shared_memory_views = shared_memory_service_->getSharedMemoryViews();
                SpPackedArray return_value = toPackedArray(std::move(points), shape, out_array, shared_memory_views, usage_flags);

                return return_value;
            });
    }

    ~NavigationService() override = default;

private:
    SharedMemoryService* shared_memory_service_ = nullptr;
};
