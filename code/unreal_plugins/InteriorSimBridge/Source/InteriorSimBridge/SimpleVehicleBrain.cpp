#include "SimpleVehicleBrain.h"

#include <string>
#include <utility>
#include <vector>

#include <EngineUtils.h>
#include <Kismet/KismetMathLibrary.h>
#include <UObject/ConstructorHelpers.h>

#include <SimpleVehicle/SimModeSimpleVehicle.h>
#include <SimpleVehicle/SimpleVehiclePawn.h>

USimpleVehicleBrain::USimpleVehicleBrain(
    const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
}

void USimpleVehicleBrain::OnActorHit(AActor* SelfActor,
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

void USimpleVehicleBrain::Init()
{
    for (TActorIterator<ASimpleVehiclePawn> it(this->GetWorld()); it; ++it)
    {
        Owner = Cast<ASimpleVehiclePawn>(*it);
    }

    check(Owner);

    Owner->OnActorHit.AddDynamic(this, &USimpleVehicleBrain::OnActorHit);

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
    std::string SimpleVehicleActionDescription =
        "The actions represent a multiplier scale applied to wheel torques.\n"
        "There are two multiplier scales.\nThe first multiplier scale moves"
        "the robot forward and backward.\nThe second multiplier scale turns "
        "the robot right and left.\n"
        "The scales are continuous in nature, ranging from [-1, 1].\n";
    unrealrl::ActionSpec SimpleVehicleActionSpec(
        false, unrealrl::DataType::Float32, {2}, std::make_pair(-1, 1),
        SimpleVehicleActionDescription);

    SetActionSpecs({SimpleVehicleActionSpec});

    std::string SimpleVehicleObservationDescription =
        "The agent has following observations.\nx-coordinate of agent w.r.t "
        "world frame. \ny-coordinate of agent w.r.t world frame.\nx-coordinate "
        "of agent relative to goal.\ny-coordinate of agent relative to goal.";
    unrealrl::ObservationSpec SimpleVehicleObservationSpec(
        {4}, unrealrl::DataType::Float32, SimpleVehicleObservationDescription);

    SetObservationSpecs({SimpleVehicleObservationSpec});
}

bool USimpleVehicleBrain::IsAgentReady()
{
    check(Owner);
    return (Owner->GetVelocity().Size() >= 0.0 &&
            Owner->GetVelocity().Size() < 0.1);
}

void USimpleVehicleBrain::OnEpisodeBegin()
{
    // reset by reload entire map
    UGameplayStatics::OpenLevel(this, FName(*GetWorld()->GetName()), false);
}

void USimpleVehicleBrain::SetAction(const std::vector<unrealrl::Action>& Action)
{
    check(Owner);

    check(Action.size() == 1);

    std::vector<float> ActionVec = Action.at(0).GetActions();

    check(ActionVec.size() == 2);

    // TODO: Should not use magic numbers
    float Scale = 10.f;
    Owner->MoveForward(Scale * ActionVec[0]);
    Owner->MoveRight(Scale * ActionVec[1]);
}

void USimpleVehicleBrain::GetObservation(
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

    // Get observation data

    ObservationVec.resize(GetObservationSpecs().size());

    // vector observations
    ObservationVec.at(0).Copy(std::vector<float>{
        0.01f * RelativePositionToTarget.X, 0.01f * RelativePositionToTarget.Y,
        0.01f * CurrentLocation.X, 0.01f * CurrentLocation.Y});

    // Reset HitInfo.
    HitInfo = UHitInfo::NoHit;
}
