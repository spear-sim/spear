//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::make_unique, std::unique_ptr
#include <string>
#include <vector>

#include "SpCore/ArrayDesc.h" // TODO: remove
#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Msgpack.h"
#include "SpServices/Rpclib.h"
#include "SpServices/Service.h"

#include "SpServices/Legacy/Agent.h"
#include "SpServices/Legacy/CameraAgent.h"
#include "SpServices/Legacy/NullAgent.h"
#include "SpServices/Legacy/SphereAgent.h"
#include "SpServices/Legacy/UrdfRobotAgent.h"
#include "SpServices/Legacy/VehicleAgent.h"

#include "SpServices/Legacy/Task.h"
#include "SpServices/Legacy/ImitationLearningTask.h"
#include "SpServices/Legacy/NullTask.h"

#include "SpServices/Legacy/NavMesh.h"

class LegacyService : public Service {
public:
    LegacyService() = delete;
    LegacyService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        unreal_entry_point_binder->bindFuncNoUnreal("legacy_service", "get_action_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(agent_);
            return agent_->getActionSpace();
        });

        unreal_entry_point_binder->bindFuncNoUnreal("legacy_service", "get_observation_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(agent_);
            return agent_->getObservationSpace();
        });

        unreal_entry_point_binder->bindFuncNoUnreal("legacy_service", "get_agent_step_info_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(agent_);
            return agent_->getStepInfoSpace();
        });

        unreal_entry_point_binder->bindFuncNoUnreal("legacy_service", "get_task_step_info_space", [this]() -> std::map<std::string, ArrayDesc> {
            SP_ASSERT(task_);
            return task_->getStepInfoSpace();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "apply_action", [this](std::map<std::string, std::vector<uint8_t>>& action) -> void {
            SP_ASSERT(agent_);
            agent_->applyAction(action);
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_observation", [this]() -> std::map<std::string, std::vector<uint8_t>> {
            SP_ASSERT(agent_);
            return agent_->getObservation();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_reward", [this]() -> float {
            SP_ASSERT(task_);
            return task_->getReward();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "is_episode_done", [this]() -> bool {
            SP_ASSERT(task_);
            return task_->isEpisodeDone();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_agent_step_info", [this]() -> std::map<std::string, std::vector<uint8_t>> {
            SP_ASSERT(agent_);
            return agent_->getStepInfo();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_task_step_info", [this]() -> std::map<std::string, std::vector<uint8_t>> {
            SP_ASSERT(task_);
            return task_->getStepInfo();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "reset_agent", [this]() -> void {
            SP_ASSERT(agent_);
            agent_->reset();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "reset_task", [this]() -> void {
            SP_ASSERT(task_);
            task_->reset();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "is_agent_ready", [this]() -> bool {
            SP_ASSERT(agent_);
            return agent_->isReady();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "is_task_ready", [this]() -> bool {
            SP_ASSERT(task_);
            return task_->isReady();
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_random_points", [this](int& num_points) -> std::vector<double> {
            SP_ASSERT(nav_mesh_);
            return nav_mesh_->getRandomPoints(num_points);
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_random_reachable_points_in_radius", [this](std::vector<double>& initial_points, float& radius) -> std::vector<double> {
            SP_ASSERT(nav_mesh_);
            return nav_mesh_->getRandomReachablePointsInRadius(initial_points, radius);
        });

        unreal_entry_point_binder->bindFuncUnreal("legacy_service", "get_paths", [this](std::vector<double>& initial_points, std::vector<double>& goal_points) -> std::vector<std::vector<double>> {
            SP_ASSERT(nav_mesh_);
            return nav_mesh_->getPaths(initial_points, goal_points);
        });

        unreal_entry_point_binder->bindFuncUnreal("unreal_service", "get_world_name", [this]() -> std::string {
            SP_ASSERT(getWorld());
            return Unreal::toStdString(getWorld()->GetName());
        });
    }

    ~LegacyService() = default;

    void worldBeginPlay() override
    {
        Service::worldBeginPlay();

        SP_ASSERT(getWorld());

        if (Config::isInitialized()) {
            if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.AGENT") == "NullAgent") {
                agent_ = std::make_unique<NullAgent>();
            } else if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.AGENT") == "CameraAgent") {
                agent_ = std::make_unique<CameraAgent>(getWorld());
            } else if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.AGENT") == "SphereAgent") {
                agent_ = std::make_unique<SphereAgent>(getWorld());
            } else if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.AGENT") == "UrdfRobotAgent") {
                agent_ = std::make_unique<UrdfRobotAgent>(getWorld());
            } else if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.AGENT") == "VehicleAgent") {
                agent_ = std::make_unique<VehicleAgent>(getWorld());
            } else {
                SP_ASSERT(false);
            }

            if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.TASK") == "NullTask") {
                task_ = std::make_unique<NullTask>();
            } else if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.TASK") == "ImitationLearningTask") {
                task_ = std::make_unique<ImitationLearningTask>(getWorld());
            } else {
                SP_ASSERT(false);
            }

        } else {
            agent_ = std::make_unique<NullAgent>();
            task_ = std::make_unique<NullTask>();
        }
        SP_ASSERT(agent_);
        SP_ASSERT(task_);

        nav_mesh_ = std::make_unique<NavMesh>();
        SP_ASSERT(nav_mesh_);

        agent_->findObjectReferences(getWorld());
        task_->findObjectReferences(getWorld());
        nav_mesh_->findObjectReferences(getWorld());

        has_world_begin_play_executed_ = true;
    }

    void worldCleanup(UWorld* world, bool session_ended, bool cleanup_resources) override
    {
        Service::worldCleanup(world, session_ended, cleanup_resources);

        SP_ASSERT(world);

        if (has_world_begin_play_executed_) {
            has_world_begin_play_executed_ = false;

            SP_ASSERT(nav_mesh_);
            SP_ASSERT(task_);
            SP_ASSERT(agent_);

            nav_mesh_->cleanUpObjectReferences();
            task_->cleanUpObjectReferences();
            agent_->cleanUpObjectReferences();

            nav_mesh_ = nullptr;
            task_ = nullptr;
            agent_ = nullptr;
        }
    }

private:
    // OpenAI Gym helper objects
    std::unique_ptr<Agent> agent_ = nullptr;
    std::unique_ptr<Task> task_ = nullptr;

    // Navmesh helper object
    std::unique_ptr<NavMesh> nav_mesh_ = nullptr;

    // Unreal life cycle state
    bool has_world_begin_play_executed_ = false;
};

//
// Enums
//

MSGPACK_ADD_ENUM(DataType);

//
// ArrayDesc
//

template <> // needed to receive a custom type as an arg
struct clmdep_msgpack::adaptor::convert<ArrayDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, ArrayDesc& array_desc) const {
        std::map<std::string, clmdep_msgpack::object> map = Msgpack::toMap(object);
        SP_ASSERT(map.size() == 6);
        array_desc.low_ = Msgpack::to<double>(map.at("low_"));
        array_desc.high_ = Msgpack::to<double>(map.at("high_"));
        array_desc.shape_ = Msgpack::to<std::vector<int64_t>>(map.at("shape_"));
        array_desc.datatype_ = Msgpack::to<DataType>(map.at("datatype_"));
        array_desc.use_shared_memory_ = Msgpack::to<bool>(map.at("use_shared_memory_"));
        array_desc.shared_memory_name_ = Msgpack::to<std::string>(map.at("shared_memory_name_"));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<ArrayDesc> {
    void operator()(clmdep_msgpack::object::with_zone& object, ArrayDesc const& array_desc) const {
        std::map<std::string, clmdep_msgpack::object> map = {
            {"low_", clmdep_msgpack::object(array_desc.low_, object.zone)},
            {"high_", clmdep_msgpack::object(array_desc.high_, object.zone)},
            {"shape_", clmdep_msgpack::object(array_desc.shape_, object.zone)},
            {"datatype_", clmdep_msgpack::object(array_desc.datatype_, object.zone)},
            {"use_shared_memory_", clmdep_msgpack::object(array_desc.use_shared_memory_, object.zone)},
            {"shared_memory_name_", clmdep_msgpack::object(array_desc.shared_memory_name_, object.zone)}};
        Msgpack::toObject(object, map);
    }
};
