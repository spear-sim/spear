#include "ImitationLearningTask.h"

#include <algorithm>

#include <EngineUtils.h>
#include <UObject/UObjectGlobals.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavigationSystem.h>

#include "ActorHitEvent.h"
#include "Assert.h"
#include "Box.h"
#include "Config.h"

ImitationLearningTask::ImitationLearningTask(UWorld* world)
{
    // append all actors that need to be ignored during collision check
    std::vector<std::string> obstacle_ignore_actor_names = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "OBSTACLE_IGNORE_ACTOR_NAMES"});

    for (TActorIterator<AActor> actor_itr(world, AActor::StaticClass()); actor_itr; ++actor_itr) {
        std::string actor_name = TCHAR_TO_UTF8(*(*actor_itr)->GetName());

        if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "AGENT_ACTOR_NAME"}) and not Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "GOAL_ACTOR_NAME"}).empty()) {
            ASSERT(!agent_actor_);
            agent_actor_ = *actor_itr;
            ASSERT(agent_actor_);
        }
        else if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"})) {
            ASSERT(!goal_actor_);
            goal_actor_ = *actor_itr;
            ASSERT(goal_actor_);
        }
        else if (std::find(obstacle_ignore_actor_names.begin(), obstacle_ignore_actor_names.end(), actor_name) != obstacle_ignore_actor_names.end()) {
            obstacle_ignore_actors_.emplace_back(*actor_itr);
        }
    }
    
    ASSERT(obstacle_ignore_actors_.size() == obstacle_ignore_actor_names.size());

    // read config value for random stream initialization
    random_stream_.Initialize(Config::getValue<int>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "RANDOM_SEED"}));

    new_object_parent_actor_ = world->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    // create and initialize actor hit handler
    actor_hit_event_ = NewObject<UActorHitEvent>(new_object_parent_actor_, TEXT("ActorHitEvent"));
    ASSERT(actor_hit_event_);
    actor_hit_event_->RegisterComponent();
    actor_hit_event_->subscribeToActor(agent_actor_);
    actor_hit_event_delegate_handle_ = actor_hit_event_->delegate_.AddRaw(this, &ImitationLearningTask::actorHitEventHandler);
    hit_goal_buffer_ = std::vector<float>(10, 0.0f);
}

ImitationLearningTask::~ImitationLearningTask()
{
    ASSERT(actor_hit_event_);
    actor_hit_event_->delegate_.Remove(actor_hit_event_delegate_handle_);
    actor_hit_event_delegate_handle_.Reset();
    actor_hit_event_->unsubscribeFromActor(agent_actor_);
    actor_hit_event_->DestroyComponent();
    actor_hit_event_ = nullptr;

    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_ = nullptr;

    random_stream_.Reset();

    // ASSERT(goal_actor_);
    goal_actor_ = nullptr;

    ASSERT(agent_actor_);
    agent_actor_ = nullptr;

    obstacle_ignore_actors_.clear();
}

void ImitationLearningTask::beginFrame()
{
    // reset hit states
    hit_goal_ = false;
    hit_obstacle_ = false;
}

void ImitationLearningTask::endFrame()
{
}

float ImitationLearningTask::getReward() const
{
    float reward;
    float normPos = agent_actor_->GetActorLocation().Size();
    
    // Hack to get the hit_goal event when the actor reaches the target location (i.e. the autopilot send a zero command for more than 10 epochs)
    hit_goal_buffer_.at(index_goal_buffer_ >= hit_goal_buffer_.size()-1 ? 0 : index_goal_buffer_++) = agent_actor_->GetActorLocation().Size();
    if (std::all_of(hit_goal_buffer_.begin(), hit_goal_buffer_.end(), [float normPos](float value) { return std::abs(value-normPos) <= 0.1; });) {
        hit_goal_ = true;
    }


    if (hit_goal_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "REWARD", "HIT_GOAL"});
    }
    else if (hit_obstacle_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "REWARD", "HIT_OBSTACLE"});
    }
    else {
        // const FVector agent_to_goal = goal_actor_->GetActorLocation() - agent_actor_->GetActorLocation();
        const FVector agent_to_goal = agent_actor_->GetActorLocation();
        reward = -agent_to_goal.Size() * Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "REWARD", "DISTANCE_TO_GOAL_SCALE"});
    }
    return reward;
}

bool ImitationLearningTask::isEpisodeDone() const
{
    return hit_goal_ or hit_obstacle_;
}

std::map<std::string, Box> ImitationLearningTask::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    Box box;

    box.low = 0;
    box.high = 1;
    box.shape = {1};
    box.dtype = DataType::Boolean;
    step_info_space["hit_goal"] = std::move(box);

    box.low = 0;
    box.high = 1;
    box.shape = {1};
    box.dtype = DataType::Boolean;
    step_info_space["hit_obstacle"] = std::move(box);

    return step_info_space;
}

std::map<std::string, std::vector<uint8_t>> ImitationLearningTask::getStepInfo() const
{
    std::map<std::string, std::vector<uint8_t>> step_info;

    step_info["hit_goal"] = std::vector<uint8_t>{hit_goal_};
    step_info["hit_obstacle"] = std::vector<uint8_t>{hit_obstacle_};

    return step_info;
}

void ImitationLearningTask::reset()
{
    float position_x, position_y, position_z;
    FVector agent_position(0), goal_position(0);

    // Spawn the agent in a random location within the navigation mesh: 
    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "SPAWN_ON_NAV_MESH"})) {

        UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(agent_actor_->GetWorld());
        auto navData = NavSys->GetMainNavData();
        ARecastNavMesh* navMesh = Cast<ARecastNavMesh>(navData);

        if (navMesh) {
            int trial = 0;
            while (trial < Config::getValue<int>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "NUMBER_RANDOM_RELOCATION_TRIALS"})) {
                FNavLocation navLocation = navMesh->GetRandomPoint();
                if (Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "ACTOR_HEIGHT"}) <= 0.0f or navLocation.Location.Z < Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "ACTOR_HEIGHT"})) {
                    agent_position = navLocation.Location;
                    return;
                }
                trial++;
            }
        }
    }

    while ((agent_position - goal_position).Size() < Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "EPISODE_BEGIN", "SPAWN_DISTANCE_THRESHOLD"})) {
        position_x = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "EPISODE_BEGIN", "AGENT_POSITION_X_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "EPISODE_BEGIN", "AGENT_POSITION_X_MAX"}));
        position_y = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "EPISODE_BEGIN", "AGENT_POSITION_Y_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "EPISODE_BEGIN", "AGENT_POSITION_Y_MAX"}));
        position_z = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "EPISODE_BEGIN", "AGENT_POSITION_Z"});

        agent_position = FVector(position_x, position_y, position_z);

        position_x = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "EPISODE_BEGIN", "GOAL_POSITION_X_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "EPISODE_BEGIN", "GOAL_POSITION_X_MAX"}));
        position_y = random_stream_.FRandRange(
            Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "EPISODE_BEGIN", "GOAL_POSITION_Y_MIN"}),
            Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "EPISODE_BEGIN", "GOAL_POSITION_Y_MAX"}));
        position_z = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "EPISODE_BEGIN", "GOAL_POSITION_Z"});

        goal_position = FVector(position_x, position_y, position_z);
    }

    // TODO: set goal location based on the navigation system...
    agent_actor_->SetActorLocation(agent_position);
    goal_actor_->SetActorLocation(goal_position);
}

bool ImitationLearningTask::isReady() const
{
    return true;
}

void ImitationLearningTask::actorHitEventHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit)
{
    ASSERT(self_actor == agent_actor_);

    if (other_actor == goal_actor_) {
        hit_goal_ = true;
    }
    else if (std::find(obstacle_ignore_actors_.begin(), obstacle_ignore_actors_.end(), other_actor) == obstacle_ignore_actors_.end()) {
        hit_obstacle_ = true;
    }
};
