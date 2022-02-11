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
    if (OtherActor && OtherActor->ActorHasTag("goal"))
    {
        HitInfo = UHitInfo::Goal;
    }
    // TODO: Does instid1227 apply to all obstacles?
    // If not, include all obstacles or provide an user interface to specify
    // obstacles
    else if (OtherActor && !OtherActor->GetName().Contains(
                               TEXT("instid1227"), ESearchCase::IgnoreCase))
    {
        HitInfo = UHitInfo::Edge;
    }
};

void USimpleVehicleBrain::Init()
{
    for (TActorIterator<ASimpleVehiclePawn> it(this->GetWorld()); it; ++it)
    {
        Owner = static_cast<ASimpleVehiclePawn*>(*it);
    }

    if (!Owner)
    {
        UE_LOG(LogTemp, Error,
               TEXT("No valid owner of this component is defined"));
        check(false);
    }

    // TODO: remove this?
    Owner->Tags.Add(TEXT("Agent"));

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

    if (!Goal)
    {
        UE_LOG(LogTemp, Error, TEXT("No valid goal found in this environment"));
        check(false);
    }

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

void USimpleVehicleBrain::SetAction(const std::vector<unrealrl::Action>& Action)
{
    check(Action.size() == 1);

    std::vector<float> ActionVec = Action.at(0).GetActions();

    check(ActionVec.size() == 2);

    check(Cast<ASimpleVehiclePawn>(Owner));

    float Scale = 10.f;
    Cast<ASimpleVehiclePawn>(Owner)->MoveForward(Scale * ActionVec[0]);
    Cast<ASimpleVehiclePawn>(Owner)->MoveRight(Scale * ActionVec[1]);
}

void USimpleVehicleBrain::GetObservation(
    std::vector<unrealrl::Observation>& ObservationVec)
{
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

bool USimpleVehicleBrain::IsAgentReady()
{
    UE_LOG(LogTemp, Warning, TEXT("Velocity is {%f}"),
           Owner->GetVelocity().Size());

    if (Owner->GetVelocity().Size() >= 0.0 && Owner->GetVelocity().Size() < 0.1)
    {
        return true;
    }
    else
    {
        return true;
    }
}

void USimpleVehicleBrain::OnEpisodeBegin()
{
    // reset by reload entire map
    UGameplayStatics::OpenLevel(this, FName(*GetWorld()->GetName()), false);
}