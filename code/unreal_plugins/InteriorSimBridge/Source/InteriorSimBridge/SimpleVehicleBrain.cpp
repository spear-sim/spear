#include "SimpleVehicleBrain.h"
#include "EngineUtils.h"
#include "Kismet/KismetMathLibrary.h"
#include "UObject/ConstructorHelpers.h"
#include "UnrealRL.h"
#include "UnrealRLManager.h"
#include "SimpleVehicle/SimModeSimpleVehicle.h"
#include "SimpleVehicle/SimpleVehiclePawn.h"

USimpleVehicleBrain::USimpleVehicleBrain(
    const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    Force = 100000.0f;
}

void USimpleVehicleBrain::OnActorHit(AActor* SelfActor,
                                     AActor* OtherActor,
                                     FVector NormalImpulse,
                                     const FHitResult& Hit)
{
    if (OtherActor && OtherActor->ActorHasTag("goal"))
    {
        UE_LOG(LogTemp, Warning, TEXT("Hit Goal"));
        HitInfo = UHitInfo::Goal;
    }
    else if (OtherActor && !OtherActor->GetName().Contains(
                               TEXT("instid1227"), ESearchCase::IgnoreCase))
    {
        UE_LOG(LogTemp, Warning, TEXT("Hit Obstacle"));
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

    Owner->Tags.Add(TEXT("Agent"));

    Owner->OnActorHit.AddDynamic(this, &USimpleVehicleBrain::OnActorHit);

    Base = Cast<UStaticMeshComponent>(Owner->GetRootComponent());

    // Store actor refs required during simulation.
    for (TActorIterator<AActor> ActorItr(GetWorld(), AActor::StaticClass());
         ActorItr; ++ActorItr)
    {
        if ((*ActorItr)->ActorHasTag("goal"))
        {
            Goal = *ActorItr;
            // break;
        }
    }

    if (!Goal)
    {
        UE_LOG(LogTemp, Error, TEXT("No valid goal found in this environment"));
        check(false);
    }

    // Initialize Observation and ActionSpec for this agent
    std::string aDescription =
        "The actions represent the velocities applied "
        "to the agent along right and left set of wheels.\nThe actions are "
        "continuous in nature.\n Values are "
        "in range [-1,1].\n";
    unrealrl::ActionSpec ActSpec(true, unrealrl::DataType::UInteger8, {1},
                                 std::make_pair(-1, 1), aDescription);

    SetActionSpecs({ActSpec});

    std::string oDescription =
        "The agent has following observations.\nx-coordinate of agent w.r.t "
        "world frame. \ny-coordinate of agent w.r.t world frame.\nx-coordinate "
        "of agent relative to goal.\ny-coordinate of agent relative to goal.";
    unrealrl::ObservationSpec ObSpec({4}, unrealrl::DataType::Float32,
                                     oDescription);

    SetObservationSpecs({ObSpec});
}

void USimpleVehicleBrain::SetAction(const std::vector<unrealrl::Action>& Action)
{
    check(Action.size() == 2);

    std::vector<float> ActionVec = Action.at(0).GetActions();

    check(ActionVec.size() == 2);

    check(Cast<ASimpleVehiclePawn>(Owner));

    // UE_LOG(LogTemp, Warning, TEXT("USimpleVehicleBrain::SetAction, %f %f"),
    // ActionVec[0], ActionVec[1]);
    /*UE_LOG(LogTemp, Warning, TEXT("Action received from client is %f"),
           ActionVec.at(0));*/
    float scale = 0.01f;
    Cast<ASimpleVehiclePawn>(Owner)->MoveForward(ActionVec[0]);
    Cast<ASimpleVehiclePawn>(Owner)->MoveRight(ActionVec[1]);
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
    UE_LOG(LogTemp, Warning, TEXT("USimpleVehicleBrain::OnEpisodeBegin start"));
    // reset by reload entire map
    UGameplayStatics::OpenLevel(this, FName(*GetWorld()->GetName()), false);

    UE_LOG(LogTemp, Warning,
           TEXT("USimpleVehicleBrain::OnEpisodeBegin complete"));
}