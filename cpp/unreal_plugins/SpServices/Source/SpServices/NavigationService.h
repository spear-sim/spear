//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t, uint64_t

#include <map>
#include <string>
#include <utility> // std::move
#include <vector>

#include <AI/Navigation/NavigationTypes.h> // FNavAgentProperties, FNavLocation, FNavPathPoint
#include <Containers/Array.h>
#include <NavigationData.h>                // FPathFindingResult
#include <NavigationSystem.h>
#include <NavigationSystemTypes.h>         // EPathFindingMode, FPathFindingQuery, FSharedConstNavQueryFilter
#include <NavFilters/NavigationQueryFilter.h>
#include <NavMesh/RecastNavMesh.h>
#include <Templates/Casts.h>
#include <UObject/Class.h>                 // UClass
#include <UObject/Object.h>                // UObject

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SpArray.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealObj.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/MsgpackAdaptors.h"
#include "SpServices/Service.h"
#include "SpServices/SharedMemoryService.h"

#include "NavigationService.generated.h"

UENUM()
enum class ESpPathFindingMode
{
    Regular      = Unreal::getConstEnumValue(EPathFindingMode::Type::Regular),
    Hierarchical = Unreal::getConstEnumValue(EPathFindingMode::Type::Hierarchical)
};

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
            [this](
                uint64_t& navigation_data,
                int& num_points,
                uint64_t& query_owner,
                std::map<std::string, SpPackedArray>& packed_arrays,
                SpPackedArray& out_array) -> SpPackedArray {

                SP_ASSERT(packed_arrays.at("filter_classes").shape_.size() == 1);
                SP_ASSERT(packed_arrays.at("filter_classes").shape_.at(0) == 0 || packed_arrays.at("filter_classes").shape_.at(0) == 1 || packed_arrays.at("filter_classes").shape_.at(0) == num_points);
                SP_ASSERT(packed_arrays.at("queriers").shape_.size() == 1);
                SP_ASSERT(packed_arrays.at("queriers").shape_.at(0) == 0 || packed_arrays.at("queriers").shape_.at(0) == 1 || packed_arrays.at("queriers").shape_.at(0) == num_points);

                ANavigationData* navigation_data_ptr = toPtr<ANavigationData>(navigation_data);
                UObject* query_owner_ptr = toPtr<UObject>(query_owner);
                SP_ASSERT(navigation_data_ptr);

                std::map<std::string, SpArraySharedMemoryView> shared_memory_views = shared_memory_service_->getSharedMemoryViews();

                SpArray<uint64_t> filter_classes("filter_classes");
                SpArray<uint64_t> queriers("queriers");
                SpArrayUtils::resolve(packed_arrays, shared_memory_views);
                SpArrayUtils::validate(packed_arrays, SpArraySharedMemoryUsageFlags::Arg);
                SpArrayUtils::moveFromPackedArrays({filter_classes.getPtr(), queriers.getPtr()}, packed_arrays);

                UClass* filter_class = nullptr;
                UObject* querier     = nullptr;

                if (filter_classes.getShape().at(0) == 1) { filter_class = toPtr<UClass>(Std::at(filter_classes.getView(), 0)); }
                if (queriers.getShape().at(0)       == 1) { querier      = toPtr<UObject>(Std::at(queriers.getView(), 0)); }

                FSharedConstNavQueryFilter filter = UNavigationQueryFilter::GetQueryFilter(*navigation_data_ptr, querier, filter_class);

                std::vector<double> points;
                for (int i = 0; i < num_points; i++) {

                    if (filter_classes.getShape().at(0) == num_points) { filter_class = toPtr<UClass>(Std::at(filter_classes.getView(), i)); }
                    if (queriers.getShape().at(0)       == num_points) { querier      = toPtr<UObject>(Std::at(queriers.getView(), i)); }

                    if (filter_classes.getShape().size() == num_points || queriers.getShape().size() == num_points) {
                        filter = UNavigationQueryFilter::GetQueryFilter(*navigation_data_ptr, querier, filter_class);
                    }

                    FVector point = navigation_data_ptr->GetRandomPoint(filter, query_owner_ptr).Location;
                    points.push_back(point.X);
                    points.push_back(point.Y);
                    points.push_back(point.Z);
                }

                return toPackedArray(std::move(points), {-1, 3}, out_array, shared_memory_views, SpArraySharedMemoryUsageFlags::Arg | SpArraySharedMemoryUsageFlags::ReturnValue);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("navigation_service", "get_random_reachable_points_in_radius",
            [this](
                uint64_t& navigation_data,
                int& num_points,
                uint64_t& query_owner,
                std::map<std::string, SpPackedArray>& packed_arrays,
                SpPackedArray& out_array) -> SpPackedArray {

                SP_ASSERT(packed_arrays.at("origin_points").shape_.size() == 2);
                SP_ASSERT(packed_arrays.at("origin_points").shape_.at(0) == 1 || packed_arrays.at("origin_points").shape_.at(0) == num_points);
                SP_ASSERT(packed_arrays.at("origin_points").shape_.at(1) == 3);
                SP_ASSERT(packed_arrays.at("radii").shape_.size() == 1);
                SP_ASSERT(packed_arrays.at("radii").shape_.at(0) == 1 || packed_arrays.at("radii").shape_.at(0) == num_points);
                SP_ASSERT(packed_arrays.at("filter_classes").shape_.size() == 1);
                SP_ASSERT(packed_arrays.at("filter_classes").shape_.at(0) == 0 || packed_arrays.at("filter_classes").shape_.at(0) == 1 || packed_arrays.at("filter_classes").shape_.at(0) == num_points);
                SP_ASSERT(packed_arrays.at("queriers").shape_.size() == 1);
                SP_ASSERT(packed_arrays.at("queriers").shape_.at(0) == 0 || packed_arrays.at("queriers").shape_.at(0) == 1 || packed_arrays.at("queriers").shape_.at(0) == num_points);

                ANavigationData* navigation_data_ptr = toPtr<ANavigationData>(navigation_data);
                UObject* query_owner_ptr = toPtr<UObject>(query_owner);
                SP_ASSERT(navigation_data_ptr);

                std::map<std::string, SpArraySharedMemoryView> shared_memory_views = shared_memory_service_->getSharedMemoryViews();

                SpArray<double> origin_points("origin_points");
                SpArray<float> radii("radii");
                SpArray<uint64_t> filter_classes("filter_classes");
                SpArray<uint64_t> queriers("queriers");
                SpArrayUtils::resolve(packed_arrays, shared_memory_views);
                SpArrayUtils::validate(packed_arrays, SpArraySharedMemoryUsageFlags::Arg);
                SpArrayUtils::moveFromPackedArrays({origin_points.getPtr(), radii.getPtr(), filter_classes.getPtr(), queriers.getPtr()}, packed_arrays);

                FVector origin       = FVector::ZeroVector;
                float radius         = -1.0f;
                UClass* filter_class = nullptr;
                UObject* querier     = nullptr;

                if (origin_points.getShape().at(0)  == 1) { origin = {Std::at(origin_points.getView(), 0), Std::at(origin_points.getView(), 1), Std::at(origin_points.getView(), 2)}; }
                if (radii.getShape().at(0)          == 1) { radius       = Std::at(radii.getView(), 0); }
                if (filter_classes.getShape().at(0) == 1) { filter_class = toPtr<UClass>(Std::at(filter_classes.getView(), 0)); }
                if (queriers.getShape().at(0)       == 1) { querier      = toPtr<UObject>(Std::at(queriers.getView(), 0)); }

                FSharedConstNavQueryFilter filter = UNavigationQueryFilter::GetQueryFilter(*navigation_data_ptr, querier, filter_class);

                std::vector<double> points;
                for (int i = 0; i < num_points; i++) {

                    if (origin_points.getShape().at(0)  == num_points) { origin       = {Std::at(origin_points.getView(), i*3 + 0), Std::at(origin_points.getView(), i*3 + 1), Std::at(origin_points.getView(), i*3 + 2)}; }
                    if (radii.getShape().at(0)          == num_points) { radius       = Std::at(radii.getView(), i); }
                    if (filter_classes.getShape().at(0) == num_points) { filter_class = toPtr<UClass>(Std::at(filter_classes.getView(), i)); }
                    if (queriers.getShape().at(0)       == num_points) { querier      = toPtr<UObject>(Std::at(queriers.getView(), i)); }

                    if (filter_classes.getShape().size() == num_points || queriers.getShape().size() == num_points) {
                        filter = UNavigationQueryFilter::GetQueryFilter(*navigation_data_ptr, querier, filter_class);
                    }

                    FNavLocation nav_location;
                    bool found = false;
                    int num_retries = 5;
                    for (int j = 0; j < num_retries; j++) {
                        found = navigation_data_ptr->GetRandomReachablePointInRadius(origin, radius, nav_location, filter, query_owner_ptr);
                        if (found) {
                            break;
                        } else {
                            SP_LOG("WARNING: ANavigationData::GetRandomReachablePointInRadius(...) couldn't find a reachable point on attempt ", j, ", retrying...");
                            SP_LOG("    origin:          ", origin.X, " ", origin.Y, " ", origin.Z);
                            SP_LOG("    radius:          ", radius);
                            SP_LOG("    filter:          ", filter.Get());
                            SP_LOG("    query_owner_ptr: ", query_owner_ptr);
                        }
                    }
                    SP_ASSERT(found);

                    points.push_back(nav_location.Location.X);
                    points.push_back(nav_location.Location.Y);
                    points.push_back(nav_location.Location.Z);
                }

                return toPackedArray(std::move(points), {-1, 3}, out_array, shared_memory_views, SpArraySharedMemoryUsageFlags::Arg | SpArraySharedMemoryUsageFlags::ReturnValue);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("navigation_service", "find_paths",
            [this](
                uint64_t& navigation_system,
                uint64_t& navigation_data,
                int& num_paths,
                uint64_t& nav_agent_interface,
                std::map<std::string, SpPackedArray>& packed_arrays,
                std::vector<std::string>& nav_agent_property_strings,
                std::vector<std::string>& path_finding_mode_strings) -> std::map<std::string, SpPackedArray> {

                SP_ASSERT(packed_arrays.at("start_points").shape_.size() == 2);
                SP_ASSERT(packed_arrays.at("start_points").shape_.at(0) == 1 || packed_arrays.at("start_points").shape_.at(0) == num_paths);
                SP_ASSERT(packed_arrays.at("start_points").shape_.at(1) == 3);
                SP_ASSERT(packed_arrays.at("end_points").shape_.size() == 2);
                SP_ASSERT(packed_arrays.at("end_points").shape_.at(0) == 1 || packed_arrays.at("end_points").shape_.at(0) == num_paths);
                SP_ASSERT(packed_arrays.at("end_points").shape_.at(1) == 3);
                SP_ASSERT(packed_arrays.at("filter_classes").shape_.size() == 1);
                SP_ASSERT(packed_arrays.at("filter_classes").shape_.at(0) == 0 || packed_arrays.at("filter_classes").shape_.at(0) == 1 || packed_arrays.at("filter_classes").shape_.at(0) == num_paths);
                SP_ASSERT(packed_arrays.at("queriers").shape_.size() == 1);
                SP_ASSERT(packed_arrays.at("queriers").shape_.at(0) == 0 || packed_arrays.at("queriers").shape_.at(0) == 1 || packed_arrays.at("queriers").shape_.at(0) == num_paths);
                SP_ASSERT(packed_arrays.at("cost_limits").shape_.size() == 1);
                SP_ASSERT(packed_arrays.at("cost_limits").shape_.at(0) == 0 || packed_arrays.at("cost_limits").shape_.at(0) == 1 || packed_arrays.at("cost_limits").shape_.at(0) == num_paths);
                SP_ASSERT(packed_arrays.at("require_navigable_end_locations").shape_.size() == 1);
                SP_ASSERT(packed_arrays.at("require_navigable_end_locations").shape_.at(0) == 0 || packed_arrays.at("require_navigable_end_locations").shape_.at(0) == 1 || packed_arrays.at("require_navigable_end_locations").shape_.at(0) == num_paths);
                SP_ASSERT(nav_agent_property_strings.size() == 0 || nav_agent_property_strings.size() == 1 || nav_agent_property_strings.size() == num_paths);
                SP_ASSERT(path_finding_mode_strings.size() == 0 || path_finding_mode_strings.size() == 1 || path_finding_mode_strings.size() == num_paths);

                UNavigationSystemV1* navigation_system_ptr = toPtr<UNavigationSystemV1>(navigation_system);
                ANavigationData* navigation_data_ptr = toPtr<ANavigationData>(navigation_data);
                INavAgentInterface* nav_agent_interface_ptr = toPtr<INavAgentInterface>(nav_agent_interface);
                SP_ASSERT(navigation_system_ptr);
                SP_ASSERT(navigation_data_ptr);

                std::map<std::string, SpArraySharedMemoryView> shared_memory_views = shared_memory_service_->getSharedMemoryViews();

                SpArray<double> start_points("start_points");
                SpArray<double> end_points("end_points");
                SpArray<uint64_t> filter_classes("filter_classes");
                SpArray<uint64_t> queriers("queriers");
                SpArray<double> cost_limits("cost_limits");
                SpArray<uint8_t> require_navigable_end_locations("require_navigable_end_locations");
                SpArrayUtils::resolve(packed_arrays, shared_memory_views);
                SpArrayUtils::validate(packed_arrays, SpArraySharedMemoryUsageFlags::Arg);
                SpArrayUtils::moveFromPackedArrays({start_points.getPtr(), end_points.getPtr(), filter_classes.getPtr(), queriers.getPtr(), cost_limits.getPtr(), require_navigable_end_locations.getPtr()}, packed_arrays);

                UnrealObj<FNavAgentProperties> nav_agent_properties;

                FVector start_point                      = FVector::ZeroVector;
                FVector end_point                        = FVector::ZeroVector;
                UClass* filter_class                     = nullptr;
                UObject* querier                         = nullptr;
                FVector::FReal cost_limit                = TNumericLimits<FVector::FReal>::Max();
                bool require_navigable_end_location      = true;
                EPathFindingMode::Type path_finding_mode = EPathFindingMode::Type::Regular;

                if (start_points.getShape().at(0)                    == 1) { start_point                    = {Std::at(start_points.getView(), 0), Std::at(start_points.getView(), 1), Std::at(start_points.getView(), 2)}; }
                if (end_points.getShape().at(0)                      == 1) { end_point                      = {Std::at(end_points.getView(), 0), Std::at(end_points.getView(), 1), Std::at(end_points.getView(), 2)}; }
                if (filter_classes.getShape().at(0)                  == 1) { filter_class                   = toPtr<UClass>(Std::at(filter_classes.getView(), 0)); }
                if (queriers.getShape().at(0)                        == 1) { querier                        = toPtr<UObject>(Std::at(queriers.getView(), 0)); }
                if (cost_limits.getShape().at(0)                     == 1) { cost_limit                     = Std::at(cost_limits.getView(), 0); }
                if (require_navigable_end_locations.getShape().at(0) == 1) { require_navigable_end_location = Std::at(require_navigable_end_locations.getView(), 0) != 0; }
                if (nav_agent_property_strings.size()                == 1) { Unreal::setObjectPropertiesFromString(nav_agent_properties.getValuePtr(), nav_agent_properties.getStaticStruct(), nav_agent_property_strings.at(0)); }
                if (path_finding_mode_strings.size()                 == 1) { path_finding_mode              = Unreal::getEnumValueFromStringAs<EPathFindingMode::Type, ESpPathFindingMode>(path_finding_mode_strings.at(0)); }

                FSharedConstNavQueryFilter filter = UNavigationQueryFilter::GetQueryFilter(*navigation_data_ptr, querier, filter_class);

                std::vector<double> points_data;
                std::vector<uint64_t> indices_data;
                int index = 0;
                for (int i = 0; i < num_paths; i++) {

                    if (start_points.getShape().at(0)                    == num_paths) { start_point                    = {Std::at(start_points.getView(), i*3 + 0), Std::at(start_points.getView(), i*3 + 1), Std::at(start_points.getView(), i*3 + 2)}; }
                    if (end_points.getShape().at(0)                      == num_paths) { end_point                      = {Std::at(end_points.getView(), i*3 + 0), Std::at(end_points.getView(), i*3 + 1), Std::at(end_points.getView(), i*3 + 2)}; }
                    if (filter_classes.getShape().at(0)                  == num_paths) { filter_class                   = toPtr<UClass>(Std::at(filter_classes.getView(), i)); }
                    if (queriers.getShape().at(0)                        == num_paths) { querier                        = toPtr<UObject>(Std::at(queriers.getView(), i)); }
                    if (cost_limits.getShape().at(0)                     == num_paths) { cost_limit                     = Std::at(cost_limits.getView(), i); }
                    if (require_navigable_end_locations.getShape().at(0) == num_paths) { require_navigable_end_location = Std::at(require_navigable_end_locations.getView(), i) != 0; }
                    if (nav_agent_property_strings.size()                == num_paths) { Unreal::setObjectPropertiesFromString(nav_agent_properties.getValuePtr(), nav_agent_properties.getStaticStruct(), nav_agent_property_strings.at(i)); }
                    if (path_finding_mode_strings.size()                 == num_paths) { path_finding_mode              = Unreal::getEnumValueFromStringAs<EPathFindingMode::Type, ESpPathFindingMode>(path_finding_mode_strings.at(i)); }

                    if (filter_classes.getShape().size() == num_paths || queriers.getShape().size() == num_paths) {
                        filter = UNavigationQueryFilter::GetQueryFilter(*navigation_data_ptr, querier, filter_class);
                    }

                    FPathFindingQuery path_finding_query;
                    if (nav_agent_interface_ptr) {
                        path_finding_query = FPathFindingQuery(*nav_agent_interface_ptr, *navigation_data_ptr, start_point, end_point, filter, nullptr, cost_limit, require_navigable_end_location);
                    } else {
                        path_finding_query = FPathFindingQuery(getWorld(), *navigation_data_ptr, start_point, end_point, filter, nullptr, cost_limit, require_navigable_end_location);
                    }

                    FPathFindingResult path_finding_result;
                    if (nav_agent_property_strings.size() == 1 || nav_agent_property_strings.size() == num_paths) {
                        path_finding_result = navigation_system_ptr->FindPathSync(nav_agent_properties.getObj(), path_finding_query, path_finding_mode);
                    } else {
                        path_finding_result = navigation_system_ptr->FindPathSync(path_finding_query, path_finding_mode);
                    }
                    SP_ASSERT(path_finding_result.IsSuccessful());
                    SP_ASSERT(path_finding_result.Path.IsValid());

                    TArray<FNavPathPoint> nav_path_points = path_finding_result.Path->GetPathPoints();
                    SP_ASSERT(nav_path_points.Num() >= 2);

                    indices_data.push_back(index);
                    for (auto& nav_path_point : nav_path_points) {
                        points_data.push_back(nav_path_point.Location.X);
                        points_data.push_back(nav_path_point.Location.Y);
                        points_data.push_back(nav_path_point.Location.Z);
                        index += 1;
                    }
                }

                SpArray<double> points("points");
                SpArray<uint64_t> indices("indices");
                points.setDataSource(std::move(points_data), {-1, 3});
                indices.setDataSource(std::move(indices_data));
                std::map<std::string, SpPackedArray> return_values = SpArrayUtils::moveToPackedArrays({points.getPtr(), indices.getPtr()});
                SpArrayUtils::validate(return_values, SpArraySharedMemoryUsageFlags::ReturnValue);

                return return_values;
            });
    }

    ~NavigationService() override = default;

private:
    SharedMemoryService* shared_memory_service_ = nullptr;
};
