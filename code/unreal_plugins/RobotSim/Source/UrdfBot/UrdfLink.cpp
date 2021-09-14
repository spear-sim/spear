#include "UrdfLink.h"
#include "UrdfBotPawn.h"

void FAUrdfLinkSecondaryTickFunction::ExecuteTick(float deltaTime, ELevelTick tickType, ENamedThreads::Type currentThread, const FGraphEventRef &myCompletionGraphEvent)
{
    if (this->target_ && !this->target_->IsPendingKill() && !this->target_->IsUnreachable())
    {
        // For profiling
        //FScopeCycleCounterUObject actorScope(this->target_);
		UE_LOG(LogTemp,Warning,TEXT("ComputeForces!"));
        this->target_->ComputeForces(deltaTime, false);
        this->target_->UpdateKinematics(deltaTime);
        this->target_->TickPostPhysics(deltaTime, tickType, *this);
    }
}

FString FAUrdfLinkSecondaryTickFunction::DiagnosticMessage()
{
    return this->target_->GetFullName() + TEXT("_TickActor");
}

AUrdfLink::AUrdfLink()
{
}

AUrdfLink::~AUrdfLink()
{
    for (auto &kvp : this->forceSpecifications_)
    {
        delete this->forceSpecifications_[kvp.Key];
        this->forceSpecifications_[kvp.Key] = nullptr;
    }
}

void AUrdfLink::BeginPlay()
{
    //Super::BeginPlay();

    this->ReleaseForceLock();

    if (!IsTemplate() && this->secondaryComponentTick_.bCanEverTick)
    {
        this->secondaryComponentTick_.target_ = this;
        this->secondaryComponentTick_.SetTickFunctionEnable(this->secondaryComponentTick_.bStartWithTickEnabled);
        //this->secondaryComponentTick_.RegisterTickFunction(owner_->GetLevel());
    }
}

void AUrdfLink::Tick(float delta)
{
    this->ComputeForces(delta, false);
    this->UpdateKinematics(delta);
}

void AUrdfLink::TickPostPhysics(float deltaSeconds, ELevelTick tickType, FAUrdfLinkSecondaryTickFunction& thisTickFunction)
{
}

void AUrdfLink::NotifyHit(class UPrimitiveComponent* myComp, class AActor* other, class UPrimitiveComponent* otherComp, bool bSelfMoved, FVector hitLocation,
    FVector hitNormal, FVector normalImpulse, const FHitResult &hit)
{
    if (this->owner_ != nullptr)
    {
        this->owner_->NotifyHit(myComp, other, otherComp, bSelfMoved, hitLocation, hitNormal, normalImpulse, hit);
    }
}

void AUrdfLink::SetCollisionComponent(UPrimitiveComponent* collisionComponent)
{
    this->collisionComponent_ = collisionComponent;
}

UPrimitiveComponent* AUrdfLink::GetCollisionComponent()
{
    return this->collisionComponent_;
}

const RobotSim::Kinematics::State& AUrdfLink::GetKinematics() const
{
    return this->kinematics_;
}

void AUrdfLink::GetReferenceFrameLocation(FVector &translation, FRotator &rotation)
{
    translation = this->referenceTranslation_;
    rotation = this->referenceRotation_;
}

void AUrdfLink::SetReferenceFrameLocation(const FVector &translation, const FRotator &rotation)
{
    this->referenceTranslation_ = translation;
    this->referenceRotation_ = rotation;
}

void AUrdfLink::RecordVisualOffset(FVector &translation, FRotator &rotation)
{
    this->visualOffsetTranslation_ = translation;
    this->visualOffsetRotation_ = rotation;
}

void AUrdfLink::SubstepTick(float deltaTime, FBodyInstance* bodyInstance)
{
    ComputeForces(deltaTime, true);
}

void AUrdfLink::AddForceSpecification(UrdfForceSpecification* forceSpecification)
{
    this->forceSpecifications_.Add(forceSpecification->Name, forceSpecification);
    this->lastSetMagnitudes_.Add(forceSpecification->Name, 0.0f);
    this->ComputeForces(0.0f, false);
}

void AUrdfLink::RemoveForceSpecification(UrdfForceSpecification* forceSpecification)
{
    FString forceSpecificationName = forceSpecification->Name;
    if (this->forceSpecifications_.Contains(forceSpecificationName))
    {
        this->AcquireForceLock();
        UrdfForceSpecification* val = this->forceSpecifications_[forceSpecificationName];
        this->forceSpecifications_.Remove(forceSpecificationName);
        this->ReleaseForceLock();
        delete forceSpecification;
    }
}

void AUrdfLink::SetForceMagnitude(FString forceName, float magnitude)
{
    if (!this->forceSpecifications_.Contains(forceName))
    {
        // TODO: Not sure if we should throw or just ignore requests for forces that don't exist.
        // For debugging purposes, we will throw. But there might be valid use cases for silently ignoring them.
        throw std::runtime_error("Request for force specification '" + std::string(TCHAR_TO_UTF8(*forceName)) + "', which does not exist.");
    }

    this->forceSpecifications_[forceName]->Magnitude = magnitude;
    this->ComputeForces(0.0f, false);
}

FVector AUrdfLink::WorldFromReferenceOffset(FVector referenceOffset, bool inputInMeters)
{
    // Get the location of the component
    FVector position = this->RootComponent->GetComponentLocation() - this->visualOffsetTranslation_;
    FRotator rotation = this->RootComponent->GetComponentRotation() - this->visualOffsetRotation_;

    // Transform the offset into the local frame
    float scale = (inputInMeters ? URobotBlueprintLib::GetWorldToMetersScale(this) : 1.0f);
    FVector worldPosition = rotation.RotateVector(referenceOffset * scale) + position;
    return worldPosition;
}

void AUrdfLink::ComputeForces(float deltaTime, bool inSubstep)
{
    unused(deltaTime);
    unused(inSubstep);

    UrdfLinearForceSpecification* linearForce = nullptr;
    UrdfAngularForceSpecification* angularForce = nullptr;

    this->AcquireForceLock();
    for (auto kvp : this->forceSpecifications_)
    {
        UrdfForceSpecification* force = kvp.Value;

        FRotator thisRotation = FRotator::ZeroRotator;
        FVector worldAxis = FVector::ZeroVector;

        switch (force->GetForceSpecificationType())
        {
            case FORCE_ANGULAR:
                angularForce = static_cast<UrdfAngularForceSpecification*>(force);
                thisRotation = this->RootComponent->GetComponentRotation();
                worldAxis = thisRotation.RotateVector(angularForce->Axis);
                this->mesh_root_->AddTorque(angularForce->Magnitude * worldAxis * this->SiTorqueToUU_);
                break;
            case FORCE_LINEAR:
                linearForce = static_cast<UrdfLinearForceSpecification*>(force);
                this->mesh_root_->AddForceAtLocation(linearForce->Axis * linearForce->Magnitude * this->SiForceToUU_, this->WorldFromReferenceOffset(linearForce->ApplicationPoint, true));
                break;
        }

        this->lastSetMagnitudes_[kvp.Key] = force->Magnitude;
    }

    this->ReleaseForceLock();

}

void AUrdfLink::SetOwningActor(AUrdfBotPawn* owner)
{
    this->owner_ = owner;
}

AUrdfBotPawn* AUrdfLink::GetOwningActor()
{
    return this->owner_;
}

TMap<FString, UrdfForceSpecification*> AUrdfLink::GetForceSpecifications() const
{
    return this->forceSpecifications_;
}

void AUrdfLink::ResetForceMagnitudes()
{
    for (auto kvp : this->forceSpecifications_)
    {
        kvp.Value->Magnitude = 0.0f;
    }
	if (this->mesh_root_)
	{
		this->ComputeForces(0.0, false);
		this->mesh_root_->SetPhysicsAngularVelocity(FVector::ZeroVector);
		this->mesh_root_->SetPhysicsLinearVelocity(FVector::ZeroVector);
	}
}

void AUrdfLink::UpdateKinematics(float dt)
{
	if (this->RootComponent != nullptr)
	{
		const auto last_kinematics = kinematics_;

		kinematics_.pose = this->GetPose();

		kinematics_.twist.linear = RobotSim::Vector3r(this->RootComponent->ComponentVelocity.X, this->RootComponent->ComponentVelocity.Y, this->RootComponent->ComponentVelocity.Z);
		kinematics_.twist.angular = RobotSim::VectorMath::toAngularVelocity(
			kinematics_.pose.orientation, last_kinematics.pose.orientation, dt);

		kinematics_.accelerations.linear = (kinematics_.twist.linear - last_kinematics.twist.linear) / dt;
		kinematics_.accelerations.angular = (kinematics_.twist.angular - last_kinematics.twist.angular) / dt;
	}
}

RobotSim::Pose AUrdfLink::GetPose()
{
    FVector position = this->GetActorLocation();
    FQuat rotation = this->GetActorRotation().Quaternion();

    auto airPosition = RobotSim::Vector3r(position.X, position.Y, position.Z);
    auto airRotation = RobotSim::Quaternionr(rotation.W, rotation.X, rotation.Y, rotation.Z);
    
    return RobotSim::Pose(airPosition, airRotation);
}

FQuat AUrdfLink::Getqua()
{
    FQuat rotation = this->GetActorRotation().Quaternion();
    return rotation;
}

void AUrdfLink::AcquireForceLock()
{
    int retryCount = 0;
    while (this->forceLock_)
    {
        FPlatformProcess::Sleep(0.001);
    }
    this->forceLock_.AtomicSet(true);
}

void AUrdfLink::ReleaseForceLock()
{
    this->forceLock_.AtomicSet(false);
}

void AUrdfLink::SetMeshFromStaticMeshComponent(UStaticMeshComponent* meshComponent)
{
    this->RootComponent = meshComponent;
    this->mesh_root_ = meshComponent;
    this->InitPostSetMesh();
}

void AUrdfLink::SetMeshFromProceduralMeshComponent(UProceduralMeshComponent* proceduralMeshComponent)
{
    this->RootComponent = proceduralMeshComponent;
    this->mesh_root_ = proceduralMeshComponent;
    this->InitPostSetMesh();
}

void AUrdfLink::InitPostSetMesh()
{
    this->onCalculateCustomPhysics.BindUObject(this, &AUrdfLink::SubstepTick);

    this->forceSpecifications_.Empty();

	if (this->mesh_root_)
	{
		this->mesh_root_->Mobility = EComponentMobility::Movable;
		this->mesh_root_->SetSimulatePhysics(true);
	}

    this->kinematics_.pose = GetPose();
    this->kinematics_.twist.linear = RobotSim::Vector3r::Zero();
    this->kinematics_.twist.angular = RobotSim::Vector3r::Zero();
    this->kinematics_.accelerations.linear = RobotSim::Vector3r::Zero();
    this->kinematics_.accelerations.angular = RobotSim::Vector3r::Zero();

    // The default Position and Velocity iterations are set very low.
    // This can cause "bouncy" joints that should be stiff, and in extreme cases can cause joints to settle in the wrong place. 
    // 10000 is an arbitrary number that appears to work well without using too much CPU.
    int32 cnt = 100000;
	if (this->mesh_root_)
	{
		this->mesh_root_->GetBodyInstance()->PositionSolverIterationCount = cnt;
		this->mesh_root_->GetBodyInstance()->VelocitySolverIterationCount = cnt;

		this->mesh_root_->SetVisibility(true);
		this->mesh_root_->RegisterComponent();
	}
}

void AUrdfLink::SetMass(float massInKg)
{
	if (this->mesh_root_)
	{
		this->mesh_root_->GetBodyInstance()->SetMassOverride(massInKg, true);
		this->mesh_root_->SetMassOverrideInKg(NAME_None, massInKg, true);
	}
}

void AUrdfLink::SetMaterial(UMaterial* material)
{
	if (this->mesh_root_)
	{
		this->mesh_root_->SetMaterial(0, material);
	}
}

UMeshComponent* AUrdfLink::GetRootMesh()
{
    return this->mesh_root_;
}

FVector AUrdfLink::GetPhysicsLinearVelocity()
{
	if (this->mesh_root_)
	{
		return this->mesh_root_->GetPhysicsLinearVelocity();
	}
	else
	{
		return FVector(0, 0, 0);
	}
}

FVector AUrdfLink::GetPhysicsAngularVelocityInRadians()
{
	if (this->mesh_root_)
	{
		return this->mesh_root_->GetPhysicsAngularVelocityInRadians();
	}
	else
	{
		return FVector(0, 0, 0);
	}
}
