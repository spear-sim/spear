#include "UrdfBotBrain.h"
#include "EngineUtils.h"
#include "Kismet/KismetMathLibrary.h"
#include "UObject/ConstructorHelpers.h"
#include "UnrealRL.h"
#include "UnrealRLManager.h"
#include "UrdfBot/UrdfBotPawn.h"
#include "UrdfBot/SimModeUrdfBot.h"
#include "ServerManager.h"

UUrdfBotBrain::UUrdfBotBrain(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    Force = 100000.0f;
}

void UUrdfBotBrain::OnActorHit(AActor* SelfActor,
                               AActor* OtherActor,
                               FVector NormalImpulse,
                               const FHitResult& Hit)
{
    if (OtherActor &&
        OtherActor->GetName().Contains(TEXT("Table"), ESearchCase::IgnoreCase))
    {
        HitInfo = UHitInfo::Goal;
    }
    else if (OtherActor && OtherActor->GetName().Contains(
                               TEXT("Wall"), ESearchCase::IgnoreCase))
    {
        HitInfo = UHitInfo::Edge;
    }
};

void UUrdfBotBrain::Init()
{
    this->SimModeOwner = Cast<AActor>(GetOwner());
    if (!SimModeOwner)
    {
        UE_LOG(LogTemp, Error,
               TEXT("No valid owner of this component is defined"));
        check(false);
    }
    auto simModeUrdfBot = Cast<ASimModeUrdfBot>(SimModeOwner);
    // simModeUrdfBot->setupVehiclesAndCamera();
    for (TActorIterator<AUrdfBotPawn> it(this->GetWorld()); it; ++it)
    {
        Owner = static_cast<AUrdfBotPawn*>(*it);
    }

    if (!Owner)
    {
        UE_LOG(LogTemp, Error,
               TEXT("No valid owner of this component is defined"));
        check(false);
    }

    Owner->Tags.Add(TEXT("Agent"));

    Owner->OnActorHit.AddDynamic(this, &UUrdfBotBrain::OnActorHit);

    Base = Cast<UStaticMeshComponent>(Owner->GetRootComponent());

    // Store actor refs required during simulation.
    for (TActorIterator<AActor> ActorItr(GetWorld(), AActor::StaticClass());
         ActorItr; ++ActorItr)
    {
        if ((*ActorItr)->GetName().Contains(TEXT("Table"),
                                            ESearchCase::IgnoreCase))
        {
            Goal = *ActorItr;
            // break;
        }
        else if ((*ActorItr)->GetName().Contains(TEXT("ViewCamera"),
                                                 ESearchCase::IgnoreCase))
        {
            ViewCamera = *ActorItr;
        }
    }

    // disable view from bot's ego centric view
    Owner->EndViewTarget(this->GetWorld()->GetFirstPlayerController());

    // Set a certain camera as this player's target view
    // TODO: Do we need to do this here?
    check(ViewCamera);
    APlayerController* Controller =
        this->GetWorld()->GetFirstPlayerController();
    if (Controller)
    {
        Controller->SetViewTarget(ViewCamera);
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
                                 std::make_pair(0, 3), aDescription);

    unrealrl::ActionSpecs ASpecs(ActSpec);
    SetActionSpecs(ASpecs);

    std::string oDescription =
        "The agent has following observations.\nx-coordinate of agent w.r.t "
        "world frame. \ny-coordinate of agent w.r.t world frame.\nx-coordinate "
        "of agent relative to goal.\ny-coordinate of agent relative to goal.";
    unrealrl::ObservationSpec ObSpec({4}, unrealrl::DataType::Float32,
                                     oDescription);

    unrealrl::ObservationSpecs ObSpecs(ObSpec);
    SetObservationSpecs(ObSpecs);
}

void UUrdfBotBrain::SetAction(const std::vector<unrealrl::Action>& Action)
{
    check(Action.size() == 1);

    std::vector<float> ActionVec = Action.at(0).GetActions();

    check(ActionVec.size() == 1);

    check(Cast<AUrdfBotPawn>(Owner));

    /*UE_LOG(LogTemp, Warning, TEXT("Action received from client is %f"),
           ActionVec.at(0));*/

    switch (static_cast<uint8>(ActionVec.at(0)))
    {
    case 0:
        Cast<AUrdfBotPawn>(Owner)->onBaseMove(1);
        break;
    case 1:
        Cast<AUrdfBotPawn>(Owner)->onBaseMove(-1);
        break;
    case 2:
        Cast<AUrdfBotPawn>(Owner)->onBaseRotate(1);
        break;
    case 3:
        Cast<AUrdfBotPawn>(Owner)->onBaseRotate(-1);
        break;
    }
}

void UUrdfBotBrain::GetObservation(
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

    ObservationVec.clear();

    // vector observations
    ObservationVec.emplace_back(unrealrl::Observation(
        {0.01f * RelativePositionToTarget.X, 0.01f * RelativePositionToTarget.Y,
         0.01f * CurrentLocation.X, 0.01f * CurrentLocation.Y}));

    // Reset HitInfo.
    HitInfo = UHitInfo::NoHit;
}

bool UUrdfBotBrain::IsAgentReady()
{
    UE_LOG(LogTemp, Warning, TEXT("Velocity is {%f}"),
           Owner->GetVelocity().Size());

    if (Owner->GetVelocity().Size() >= 0.0 && Owner->GetVelocity().Size() < 0.1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void UUrdfBotBrain::OnEpisodeBegin()
{
    UE_LOG(LogTemp, Warning, TEXT("UUrdfBotBrain::ResetAgent"));
    UGameplayStatics::OpenLevel(this, FName(*GetWorld()->GetName()), false);
}
