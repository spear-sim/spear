#include "ImitationLearningTask.h"

#include <algorithm>
#include <iostream>
#include <EngineUtils.h>
#include <UObject/UObjectGlobals.h>

#include "ActorHitEvent.h"
#include "Assert.h"
#include "Box.h"
#include "Config.h"

ImitationLearningTask::ImitationLearningTask(UWorld* world)
{
    std::vector<std::string> obstacle_ignore_actor_names = Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "OBSTACLE_IGNORE_ACTOR_NAMES"});

    for (TActorIterator<AActor> actor_itr(world, AActor::StaticClass()); actor_itr; ++actor_itr) {

        std::string actor_name = TCHAR_TO_UTF8(*(*actor_itr)->GetName());

        if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "AGENT_ACTOR_NAME"})) {
            ASSERT(!agent_actor_);
            agent_actor_ = *actor_itr;
            ASSERT(agent_actor_);
        }
        else if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"}) and Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"}) != "") {
            ASSERT(!goal_actor_);
            goal_actor_ = *actor_itr;;
            ASSERT(goal_actor_);
        }
        else if (std::find(obstacle_ignore_actor_names.begin(), obstacle_ignore_actor_names.end(), actor_name) != obstacle_ignore_actor_names.end()) {
            
            obstacle_ignore_actors_.emplace_back(*actor_itr);
            std::cout << "actor_name: " << actor_name << std::endl;
        }
    }


    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "GOAL_ACTOR_NAME"}) == "") {
        ASSERT(!goal_actor_);
        goal_actor_ = world->SpawnActor<AActor>();
        ASSERT(goal_actor_);
        goal_actor_hit_event_ = NewObject<UActorHitEvent>(goal_actor_, TEXT("GoalHitEvent"));
        ASSERT(goal_actor_hit_event_);
        goal_actor_hit_event_->RegisterComponent();
    }
    
    //ASSERT(obstacle_ignore_actors_.size() == obstacle_ignore_actor_names.size());

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

    ASSERT(goal_actor_);
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

    // TEMPORARY HACK !!
    APawn* vehicle_pawn = dynamic_cast<APawn*>(agent_actor_);
    ASSERT(vehicle_pawn);
    if (Navigation::Singleton(vehicle_pawn).goalReached())
    {
        hit_goal_ = true;
    }

    if (hit_goal_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "REWARD", "HIT_GOAL"});
    }
    else if (hit_obstacle_) {
        reward = Config::getValue<float>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "REWARD", "HIT_OBSTACLE"});
    }
    else {
        const FVector agent_to_goal = goal_actor_->GetActorLocation() - agent_actor_->GetActorLocation();
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
    FVector agent_position(0), goal_position(0);

    APawn* vehicle_pawn = dynamic_cast<APawn*>(agent_actor_);
    ASSERT(vehicle_pawn);

    Navigation::Singleton(vehicle_pawn).resetNavigation();

    if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "IMITATION_LEARNING_TASK", "RANDOM_SPAWN_TRAJ"}) == true) {
        // Random initial position:
        agent_position = Navigation::Singleton(vehicle_pawn).generateRandomInitialPosition();
        agent_actor_->SetActorLocation(agent_position);

        // Trajectory planning:
        Navigation::Singleton(vehicle_pawn).generateTrajectoryToRandomTarget();
    }
    else {
        // Predefined initial position:
        agent_position = Navigation::Singleton(vehicle_pawn).getPredefinedInitialPosition(); // DIRTY HACK for neurips
        agent_actor_->SetActorLocation(agent_position);

        // Trajectory planning:
        Navigation::Singleton(vehicle_pawn).generateTrajectoryToPredefinedTarget();
    }

    goal_position = Navigation::Singleton(vehicle_pawn).getGoal();
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
