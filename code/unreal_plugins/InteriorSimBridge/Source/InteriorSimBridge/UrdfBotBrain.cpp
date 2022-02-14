#include "UrdfBotBrain.h"

#include <string>
#include <utility>
#include <vector>

#include <EngineUtils.h>
#include <Kismet/KismetMathLibrary.h>
#include <UObject/ConstructorHelpers.h>

#include <UrdfBot/UrdfBotPawn.h>
#include <UrdfBot/SimModeUrdfBot.h>

UUrdfBotBrain::UUrdfBotBrain(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
}

void UUrdfBotBrain::OnActorHit(AActor* SelfActor,
                               AActor* OtherActor,
                               FVector NormalImpulse,
                               const FHitResult& Hit)
{
    check(OtherActor);
    if (OtherActor->ActorHasTag("goal"))
    {
        HitInfo = UHitInfo::Goal;
    }
    // TODO: Does instid1227 apply to all obstacles?
    // If not, include all obstacles or provide an user interface to specify
    // obstacles
    else if (!OtherActor->GetName().Contains(TEXT("instid1227"),
                                             ESearchCase::IgnoreCase))
    {
        HitInfo = UHitInfo::Edge;
    }
};

void UUrdfBotBrain::Init()
{
    for (TActorIterator<AUrdfBotPawn> it(this->GetWorld()); it; ++it)
    {
        Owner = Cast<AUrdfBotPawn>(*it);
    }

    check(Owner);

    Owner->OnActorHit.AddDynamic(this, &UUrdfBotBrain::OnActorHit);

    // Store actor refs required during simulation.
    for (TActorIterator<AActor> ActorItr(GetWorld(), AActor::StaticClass());
         ActorItr; ++ActorItr)
    {
        if ((*ActorItr)->ActorHasTag("goal"))
        {
            Goal = *ActorItr;
            break;
        }
    }

    check(Goal);

    // Initialize ObservationSpec and ActionSpec for this agent
    std::string UrdfBotActionDescription =
        "The actions represents moving the robot in 4 directions - Forward, "
        "Backward, turn right, turn left.\n"
        "Actions are discrete in nature.\n There are 4 possible values, "
        "ranging from 0 to 3.\n"
        "0 - move forward\n"
        "1 - move backwards\n"
        "2 - rotate right\n"
        "3 - rotate left.\n";
    unrealrl::ActionSpec UrdfBotActionSpec(true, unrealrl::DataType::UInteger8,
                                           {1}, std::make_pair(0, 3),
                                           UrdfBotActionDescription);

    SetActionSpecs({UrdfBotActionSpec});

    std::string UrdfBotObservationDescription =
        "The agent has following observations.\nx-coordinate of agent w.r.t "
        "world frame. \ny-coordinate of agent w.r.t world frame.\nx-coordinate "
        "of agent relative to goal.\ny-coordinate of agent relative to goal.";
    unrealrl::ObservationSpec UrdfBotObservationSpec(
        {4}, unrealrl::DataType::Float32, UrdfBotObservationDescription);

    SetObservationSpecs({UrdfBotObservationSpec});
}

bool UUrdfBotBrain::IsAgentReady()
{
    check(Owner);
    return (Owner->GetVelocity().Size() >= 0.0 &&
            Owner->GetVelocity().Size() < 0.1);
}

void UUrdfBotBrain::OnEpisodeBegin()
{
    // reset by reload entire map
    UGameplayStatics::OpenLevel(this, FName(*GetWorld()->GetName()), false);
}

void UUrdfBotBrain::SetAction(const std::vector<unrealrl::Action>& Action)
{
    check(Action.size() == 1);

    std::vector<float> ActionVec = Action.at(0).GetActions();

    check(ActionVec.size() == 1);
    check(Owner);

    switch (static_cast<uint8>(ActionVec.at(0)))
    {
    case 0:
        Owner->onBaseMove(1);
        break;
    case 1:
        Owner->onBaseMove(-1);
        break;
    case 2:
        Owner->onBaseRotate(1);
        break;
    case 3:
        Owner->onBaseRotate(-1);
        break;
    }
}

void UUrdfBotBrain::GetObservation(
    std::vector<unrealrl::Observation>& ObservationVec)
{
    check(Owner);
    check(Goal);

    // Get observations.
    const FVector CurrentLocation = Owner->GetActorLocation();

    const FVector RelativePositionToTarget(
        (Goal->GetActorLocation() - CurrentLocation).X,
        (Goal->GetActorLocation() - CurrentLocation).Y, 0);

    // Scaled distance to goal.
    SetCurrentReward(-RelativePositionToTarget.Size() / 100000);

    if (HitInfo == UHitInfo::Goal)
    {
        SetCurrentReward(100);
        EndEpisode();
    }
    else if (HitInfo == UHitInfo::Edge)
    {
        SetCurrentReward(-100);
        EndEpisode();
    }

    ObservationVec.resize(GetObservationSpecs().size());

    // vector observations
    ObservationVec.at(0).Copy(std::vector<float>{
        0.01f * RelativePositionToTarget.X, 0.01f * RelativePositionToTarget.Y,
        0.01f * CurrentLocation.X, 0.01f * CurrentLocation.Y});

    // Reset HitInfo.
    HitInfo = UHitInfo::NoHit;
}
