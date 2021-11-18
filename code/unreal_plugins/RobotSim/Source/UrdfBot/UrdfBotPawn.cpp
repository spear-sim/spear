#include "UrdfBotPawn.h"
#include "Engine/Engine.h"
#include "PhysicsEngine/PhysicsHandleComponent.h"
#include "SimModeUrdfBot.h"

AUrdfBotPawn::AUrdfBotPawn()
{
    static ConstructorHelpers::FObjectFinder<UStaticMesh> baseBoxMesh(
        TEXT("/Engine/BasicShapes/Cube.Cube"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> baseCylinderMesh(
        TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> baseSphereMesh(
        TEXT("/Engine/BasicShapes/Sphere.Sphere"));

    if (baseBoxMesh.Succeeded())
        this->baseBoxMesh_ = baseBoxMesh.Object;

    if (baseCylinderMesh.Succeeded())
        this->baseCylinderMesh_ = baseCylinderMesh.Object;

    if (baseSphereMesh.Succeeded())
        this->baseSphereMesh_ = baseSphereMesh.Object;

    // construct UphysicsHandlecomponent
    mPhysicsHandle = CreateDefaultSubobject<UPhysicsHandleComponent>(
        FName("PhysicsHandleComponent"));
    mIsClawGrabbed = false;

    // Create a default root component. Needed for spawning to work properly.
    auto DefaultCapsule =
        CreateDefaultSubobject<UCapsuleComponent>(FName("Default"));
    DefaultCapsule->InitCapsuleSize(0.01f, 0.01f);
    DefaultCapsule->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    DefaultCapsule->SetCollisionResponseToAllChannels(ECR_Ignore);
    DefaultCapsule->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
    DefaultCapsule->SetVisibility(false);
    RootComponent = DefaultCapsule;

    // Parse the URDF file to get a list of materials and static meshes.
    // These can only be created in the constructor as far as I can tell.
    // Cache them for use later.
    //
    // Also, the unreal editor calls the constructor. At this point, the
    // settings file is not loaded, which happens during SimMode initialization.
    // This causes the UrdfPath to be null, throwing an exception. For now,
    // catch that exception. If the path is improper, BeginPlay() will throw.
    try
    {
        RobotSimSettings& settings =
            RobotSimSettings::RobotSimSettings::singleton();
        if (settings.pawn_paths.find("UrdfBot") == settings.pawn_paths.end())
            throw std::runtime_error(
                "Cannot find pawn path. Make sure settings.json has a member "
                "inside 'PawnPaths' named 'Urdfbot'.");

        FString urdfPath =
            FString(settings.pawn_paths["UrdfBot"].urdf_path.c_str());
        if (urdfPath.Len() == 0)
            throw std::runtime_error(
                "No pawn path specified. Make sure that the 'PawnPaths' member "
                "named 'UrdfBot' has member 'urdf_path' specified.");
        if (!FPaths::FileExists(urdfPath))
        {
            throw std::runtime_error(
                "Invalid pawn_paths. Make sure that the UrdfFile exists");
        }
        this->components_.Empty();

        UrdfParser parser;
        parser.Parse(urdfPath);

        this->materials_.Empty();

        for (auto kvp : parser.GetMaterials())
        {
            FString materialName = kvp.Value->Name;
            FString materialPath = kvp.Value->TextureFile;
            if (materialPath.Len() > 0)
            {
                ConstructorHelpers::FObjectFinder<UMaterialInterface> material(
                    *materialPath);

                if (material.Object != NULL)
                {
                    this->materials_.Add(
                        materialName,
                        static_cast<UMaterialInterface*>(material.Object));
                }
            }
        }

        this->user_static_meshes_.Empty();
        for (auto kvp : parser.GetLinks())
        {
            UrdfMesh* linkGeometry = nullptr;
            UrdfLinkVisualSpecification* visualSpecification =
                kvp.Value->VisualSpecification;
            UrdfLinkCollisionSpecification* collisionSpecification =
                kvp.Value->CollisionSpecification;

            if (visualSpecification != nullptr &&
                visualSpecification->Geometry->GetGeometryType() == MESH)
            {
                linkGeometry =
                    static_cast<UrdfMesh*>(visualSpecification->Geometry);
            }
            else if (collisionSpecification != nullptr &&
                     collisionSpecification->Geometry->GetGeometryType() ==
                         MESH)
            {
                linkGeometry =
                    static_cast<UrdfMesh*>(visualSpecification->Geometry);
            }

            if (linkGeometry == nullptr ||
                linkGeometry->FileType != UNREAL_MESH)
            {
                continue;
            }

            ConstructorHelpers::FObjectFinder<UStaticMesh> mesh(
                *linkGeometry->FileLocation);

            if (mesh.Object != NULL)
            {
                this->user_static_meshes_.Add(
                    linkGeometry->FileLocation,
                    static_cast<UStaticMesh*>(mesh.Object));
            }
        }

        this->staticMeshGenerator_.Initialize(
            this->baseBoxMesh_, this->baseCylinderMesh_, this->baseSphereMesh_,
            this->user_static_meshes_);
    }
    catch (std::exception e)
    {
        int j = 0;
    }
}

AUrdfBotPawn::~AUrdfBotPawn()
{
}

void AUrdfBotPawn::BeginPlay()
{
    Super::BeginPlay();
}

void AUrdfBotPawn::Tick(float delta)
{
    Super::Tick(delta);

    for (const auto& kvp : this->controlled_motion_components_)
    {
        kvp.Value->ComputeForces(delta);
        // kvp.Value->GetState();
    }

    for (const auto& kvp : this->components_)
    {
        kvp.Value->Tick(delta);
    }

    this->pawn_events_.getPawnTickSignal().emit(delta);

    if (this->draw_debug_)
    {
        this->DrawDebug();
    }
    // update physicshandle location
    if (this->mEndEffectorLink)
    {
        FVector clawLocation = this->mEndEffectorLink->GetActorLocation();
        this->mPhysicsHandle->SetTargetLocation(clawLocation);
    }
}

void AUrdfBotPawn::EndPlay(const EEndPlayReason::Type endPlayReason)
{
    for (auto& kvp : this->constraints_)
    {
        UPhysicsConstraintComponent* component = kvp.Value.Value;
        component->BreakConstraint();
        // component->DestroyComponent();
    }
}

void AUrdfBotPawn::NotifyHit(class UPrimitiveComponent* myComp,
                             class AActor* other,
                             class UPrimitiveComponent* otherComp,
                             bool bSelfMoved,
                             FVector hitLocation,
                             FVector hitNormal,
                             FVector normalImpulse,
                             const FHitResult& hit)
{
    // myComp is the hit link
    // other is the actor that is being hit

    if (!IsValid(myComp))
    {
        return;
    }

    if (!IsValid(other))
    {
        return;
    }

    if (!IsValid(otherComp))
    {
        return;
    }

    FString meshCompName = myComp->GetName();
    FString otherCompName = other->GetName();
    if (this->collision_blacklist_.Contains(meshCompName))
    {
        TArray<FRegexPattern> patterns =
            this->collision_blacklist_[meshCompName];
        for (int i = 0; i < patterns.Num(); i++)
        {
            FRegexMatcher matcher(patterns[i], otherCompName);
            if (matcher.FindNext())
            {
                return;
            }
        }
    }

    this->pawn_events_.getCollisionSignal().emit(myComp, other, otherComp,
                                                 bSelfMoved, hitLocation,
                                                 hitNormal, normalImpulse, hit);
}

void AUrdfBotPawn::InitializeForBeginPlay()
{
    RobotSimSettings& settings =
        RobotSimSettings::RobotSimSettings::singleton();
    if (settings.pawn_paths.find("UrdfBot") == settings.pawn_paths.end())
        throw std::runtime_error(
            "Cannot find pawn path. Make sure settings.json has a member "
            "inside 'PawnPaths' named 'Urdfbot'.");

    FString urdfPath =
        FString(settings.pawn_paths["UrdfBot"].urdf_path.c_str());
    if (urdfPath.Len() == 0)
        throw std::runtime_error(
            "No pawn path specified. Make sure that the 'PawnPaths' member "
            "named 'UrdfBot' has member 'urdf_path' specified.");
    if (!FPaths::FileExists(urdfPath))
    {
        throw std::runtime_error(
            "Invalid pawn_paths. Make sure that the UrdfFile exists");
    }
    // TODO: change when multiple vehicles are supported.
    RobotSimSettings::VehicleSetting* vehicleSetting =
        settings.vehicles["UrdfBot"].get();
    this->debug_symbol_scale_ = vehicleSetting->debug_symbol_scale;
    this->draw_debug_ =
        !RobotSim::Utils::isApproximatelyZero(this->debug_symbol_scale_);

    if (this->draw_debug_ && this->debug_symbol_scale_ < 0)
    {
        throw std::runtime_error(
            "Debug symbol scale is < 0. Must be a positive number.");
    }

    this->scale_factor_ = settings.pawn_paths["UrdfBot"].scale_factor;

    this->ConstructFromFile(urdfPath);

    std::string& urdf_path = settings.pawn_paths["UrdfBot"].urdf_path;
    this->inverseKinematicComponent->ImportModel(urdf_path,
                                                 this->scale_factor_);

    // set left wheel joint
    FString leftWheelJoint =
        vehicleSetting->controlSetting.LeftWheelJoint.c_str();
    if (leftWheelJoint.Len() > 0)
    {
        if (this->controlled_motion_components_.Contains(leftWheelJoint))
        {
            this->mLeftWheelJoint = static_cast<Motor*>(
                this->controlled_motion_components_[leftWheelJoint]);
            this->mLeftWheelJoint->SetDrive(0, 1000);
        }
    }
    // Set right wheel joint
    FString rightWheelJoint =
        vehicleSetting->controlSetting.RightWheelJoint.c_str();
    if (rightWheelJoint.Len() > 0)
    {
        if (this->controlled_motion_components_.Contains(rightWheelJoint))
        {
            this->mRightWheelJoint = static_cast<Motor*>(
                this->controlled_motion_components_[rightWheelJoint]);
            this->mRightWheelJoint->SetDrive(0, 1000);
        }
    }
    // set manipulator joint
    std::vector<std::string> manipulatorJoints =
        vehicleSetting->controlSetting.ManipulatorJoints;
    if (manipulatorJoints.size() > 0)
    {
        for (size_t i = 0; i < manipulatorJoints.size(); i++)
        {
            FString manipulatorJoint = manipulatorJoints[i].c_str();
            if (this->controlled_motion_components_.Contains(manipulatorJoint))
            {
                this->mManipulatorJointList.Add(
                    this->controlled_motion_components_[manipulatorJoint]);
                this->controlled_motion_components_[manipulatorJoint]->SetDrive(
                    60000, 5);
            }
        }
    }
    // set end effector link
    FString endEffectorName =
        vehicleSetting->controlSetting.EndEffectorLink.c_str();
    if (endEffectorName.Len() > 0)
    {
        if (this->components_.Contains(endEffectorName))
        {
            this->setEndEffector(FString(endEffectorName));
        }
    }

    this->collision_blacklist_.Empty();
    for (auto& kvp : vehicleSetting->collision_blacklist)
    {
        FString bot_mesh = FString(kvp.first.c_str()) + TEXT("_visual");
        FString regex = FString(kvp.second.c_str());

        if (!this->collision_blacklist_.Contains(bot_mesh))
        {
            this->collision_blacklist_.Add(bot_mesh, TArray<FRegexPattern>());
        }

        this->collision_blacklist_[bot_mesh].Add(FRegexPattern(regex));
    }

    if (settings.pawn_paths["UrdfBot"].enable_keyboard)
    {
        setupInputBindings();
    }
    // must be initialized
    InitializeForCatchObject();
}

common_utils::UniqueValueMap<std::string, APIPCamera*>
AUrdfBotPawn::GetCameras() const
{
    return this->cameras_;
}

PawnEvents* AUrdfBotPawn::GetPawnEvents()
{
    return &(this->pawn_events_);
}

TMap<FString, AUrdfLink*> AUrdfBotPawn::GetLinkComponents() const
{
    return this->components_;
}

AUrdfLink* AUrdfBotPawn::GetRootLinkComponent() const
{
    return this->root_component_;
}

TMap<FString, ControlledMotionComponent*>
AUrdfBotPawn::GetControlledMotionComponents() const
{
    return this->controlled_motion_components_;
}

TMap<FString, ControlledMotionComponent*>
AUrdfBotPawn::GetBotMotionComponents() const
{
    return this->movement_components_;
}

void AUrdfBotPawn::TeleportToLocation(FVector position,
                                      FQuat orientation,
                                      bool teleport)
{
    FVector translation =
        (position * URobotBlueprintLib::GetWorldToMetersScale(this)) -
        this->GetActorLocation();
    FRotator rotation =
        (orientation * this->GetActorQuat().Inverse()).Rotator();

    this->MoveAllComponents(translation, rotation);

    RobotBase::TeleportToLocation(
        position * URobotBlueprintLib::GetWorldToMetersScale(this), orientation,
        teleport);
}

void AUrdfBotPawn::setupInputBindings()
{
    URobotBlueprintLib::EnableInput(this);

    float val = 4;
    float rotVal = 2;
    // keyboard control over xxx
    URobotBlueprintLib::BindAxisToKey(
        FInputAxisKeyMapping("onBaseMove", EKeys::W, val), this, this,
        &AUrdfBotPawn::onBaseMove);
    URobotBlueprintLib::BindAxisToKey(
        FInputAxisKeyMapping("onBaseMove", EKeys::S, -val), this, this,
        &AUrdfBotPawn::onBaseMove);
    URobotBlueprintLib::BindAxisToKey(
        FInputAxisKeyMapping("onBaseRotate", EKeys::A, rotVal), this, this,
        &AUrdfBotPawn::onBaseRotate);
    URobotBlueprintLib::BindAxisToKey(
        FInputAxisKeyMapping("onBaseRotate", EKeys::D, -rotVal), this, this,
        &AUrdfBotPawn::onBaseRotate);

    URobotBlueprintLib::BindAxisToKey(
        FInputAxisKeyMapping("onBaseMove", EKeys::Up, val), this, this,
        &AUrdfBotPawn::onBaseMove);
    URobotBlueprintLib::BindAxisToKey(
        FInputAxisKeyMapping("onBaseMove", EKeys::Down, -val), this, this,
        &AUrdfBotPawn::onBaseMove);
    URobotBlueprintLib::BindAxisToKey(
        FInputAxisKeyMapping("onBaseRotate", EKeys::Left, rotVal), this, this,
        &AUrdfBotPawn::onBaseRotate);
    URobotBlueprintLib::BindAxisToKey(
        FInputAxisKeyMapping("onBaseRotate", EKeys::Right, -rotVal), this, this,
        &AUrdfBotPawn::onBaseRotate);

    // keyboard binding to move prismatic joint
    URobotBlueprintLib::BindActionToKey("onStiffnessUp", EKeys::E, this,
                                        &AUrdfBotPawn::onStiffnessUp, true);
    URobotBlueprintLib::BindActionToKey("onStiffnessDown", EKeys::Q, this,
                                        &AUrdfBotPawn::onStiffnessDown, true);

    URobotBlueprintLib::BindActionToKey("onTorsoUp", EKeys::T, this,
                                        &AUrdfBotPawn::onTorsoUp, true);
    URobotBlueprintLib::BindActionToKey("onTorsoDown", EKeys::G, this,
                                        &AUrdfBotPawn::onTorsoDown, true);

    URobotBlueprintLib::BindActionToKey("onArmXUp", EKeys::I, this,
                                        &AUrdfBotPawn::onArmXUp, true);
    URobotBlueprintLib::BindActionToKey("onArmXDown", EKeys::K, this,
                                        &AUrdfBotPawn::onArmXDown, true);

    URobotBlueprintLib::BindActionToKey("onArmYUp", EKeys::L, this,
                                        &AUrdfBotPawn::onArmYUp, true);
    URobotBlueprintLib::BindActionToKey("onArmYDown", EKeys::J, this,
                                        &AUrdfBotPawn::onArmYDown, true);

    URobotBlueprintLib::BindActionToKey("onArmZUp", EKeys::Y, this,
                                        &AUrdfBotPawn::onArmZUp, true);
    URobotBlueprintLib::BindActionToKey("onArmZDown", EKeys::H, this,
                                        &AUrdfBotPawn::onArmZDown, true);

    URobotBlueprintLib::BindActionToKey("onArmXRotationUp", EKeys::Z, this,
                                        &AUrdfBotPawn::onArmXRotationUp, true);
    URobotBlueprintLib::BindActionToKey("onArmXRotationDown", EKeys::C, this,
                                        &AUrdfBotPawn::onArmXRotationDown,
                                        true);

    URobotBlueprintLib::BindActionToKey("onArmYRotationUp", EKeys::V, this,
                                        &AUrdfBotPawn::onArmYRotationUp, true);
    URobotBlueprintLib::BindActionToKey("onArmYRotationDown", EKeys::B, this,
                                        &AUrdfBotPawn::onArmYRotationDown,
                                        true);

    URobotBlueprintLib::BindActionToKey("onArmZRotationUp", EKeys::N, this,
                                        &AUrdfBotPawn::onArmZRotationUp, true);
    URobotBlueprintLib::BindActionToKey("onArmZRotationDown", EKeys::M, this,
                                        &AUrdfBotPawn::onArmZRotationDown,
                                        true);

    URobotBlueprintLib::BindActionToKey("onArmReset", EKeys::R, this,
                                        &AUrdfBotPawn::onArmReset, true);
    URobotBlueprintLib::BindActionToKey("onTest", EKeys::F, this,
                                        &AUrdfBotPawn::onTest, true);

    URobotBlueprintLib::BindActionToKey("onManipulator", EKeys::SpaceBar, this,
                                        &AUrdfBotPawn::onManipulator, true);

    ////运动控制绑定
    // URobotBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward",
    // EKeys::Up, 0.2), this, 	this, &AUrdfBotPawn::onMoveForward);

    // URobotBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward",
    // EKeys::Down, 0.0), this, 	this, &AUrdfBotPawn::onMoveForward);

    // URobotBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight",
    // EKeys::Right, 0.5), this, 	this, &AUrdfBotPawn::onMoveRight);

    // URobotBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight",
    // EKeys::Left, -0.5), this, 	this, &AUrdfBotPawn::onMoveRight);

    // URobotBlueprintLib::BindActionToKey("CatchObject", EKeys::U , this,
    // &AUrdfBotPawn::onGrabObject, true);
    // URobotBlueprintLib::BindActionToKey("Handbrake", EKeys::End, this,
    // &AUrdfBotPawn::onDropReleased, false);

    // URobotBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake",
    // EKeys::SpaceBar, 1), this, 	this, &AUrdfBotPawn::onFootBrake);

    // URobotBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight",
    // EKeys::Gamepad_LeftX, 1), this, 	this, &AUrdfBotPawn::onMoveRight);

    // URobotBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward",
    // EKeys::Gamepad_RightTriggerAxis, 1), this, 	this,
    //&AUrdfBotPawn::onMoveForward);

    // URobotBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake",
    // EKeys::Gamepad_LeftTriggerAxis, 1), this, 	this,
    //&AUrdfBotPawn::onFootBrake);
}

void AUrdfBotPawn::onMoveForward(float Val)
{
    TArray<FString> components;
    components.Push("front_right_wheel");
    components.Push("front_left_wheel");
    components.Push("rear_right_wheel");
    components.Push("rear_left_wheel");

    GEngine->AddOnScreenDebugMessage(0, 1.0f, FColor::Red,
                                     *FString::SanitizeFloat(Val));
    for (auto& componentName : components)
    {
        if (controlled_motion_components_.Contains(componentName))
        {
            ControlledMotionComponent* component =
                controlled_motion_components_[componentName];
            TMap<FString, float> controlSignalValues;
            controlSignalValues.Add(FString("Value"), Val);
            component->SetControl(controlSignalValues);
        }
    }
}

void AUrdfBotPawn::onMoveRight(float Val)
{
    //小于0的时候，向左转。
    if (Val < 0)
    {

        TArray<FString> components;
        components.Push("front_right_wheel");
        components.Push("rear_right_wheel");

        for (auto& componentName : components)
        {
            if (controlled_motion_components_.Contains(componentName))
            {
                ControlledMotionComponent* component =
                    controlled_motion_components_[componentName];
                TMap<FString, float> controlSignalValues;
                controlSignalValues.Add(FString("Value"), Val);
                component->SetControl(controlSignalValues);
            }
        }
    }
    if (Val > 0)
    {
        TArray<FString> components;
        components.Push("front_left_wheel");
        components.Push("rear_left_wheel");

        for (auto& componentName : components)
        {
            if (controlled_motion_components_.Contains(componentName))
            {
                ControlledMotionComponent* component =
                    controlled_motion_components_[componentName];
                TMap<FString, float> controlSignalValues;
                controlSignalValues.Add(FString("Value"), -Val);
                component->SetControl(controlSignalValues);
            }
        }
    }
}

void AUrdfBotPawn::onGrabObject()
{
    if (!this->mEndEffectorLink)
    {
        return;
    }
    FTransform result =
        getRelativePoseByLink(this->root_component_, this->mEndEffectorLink);
    URobotBlueprintLib::LogMessage(FString("end effector link transform: \n"),
                                   *result.ToHumanReadableString(),
                                   LogDebugLevel::Informational, 30);

    if (!mIsClawGrabbed && mOtherHitComponent)
    {
        this->mPhysicsHandle->GrabComponentAtLocation(mOtherHitComponent,
                                                      FName(), this->mHitPoint);
        mIsClawGrabbed = true;
    }
    else
    {
        this->mPhysicsHandle->ReleaseComponent();
        mOtherHitComponent = nullptr;
        mIsClawGrabbed = false;
    }
}

bool AUrdfBotPawn::InitializeForCatchObject()
{
    if (this->mEndEffectorLink)
    {
        // add callback function for end effector
        UStaticMeshComponent* rootMesh = static_cast<UStaticMeshComponent*>(
            this->mEndEffectorLink->GetRootMesh());
        rootMesh->OnComponentHit.AddDynamic(
            this, &AUrdfBotPawn::OnEndEffectorCollision);
    }

    return true;
}

void AUrdfBotPawn::OnEndEffectorCollision(UPrimitiveComponent* HitComponent,
                                          AActor* OtherActor,
                                          UPrimitiveComponent* OtherComp,
                                          FVector NormalImpulse,
                                          const FHitResult& Hit)
{
    if (OtherActor->GetName() != FString("UrdfBot"))
    {
        GEngine->AddOnScreenDebugMessage(
            1, 8.f, FColor::Red, TEXT("please press 'U' to catch object"));
        URobotBlueprintLib::LogMessage(FString("OnClawCollision"),
                                       *OtherActor->GetName(),
                                       LogDebugLevel::Failure, 30);

        AActor* hitActor = Hit.GetActor();
        if (hitActor && hitActor->GetRootComponent())
        {
            mOtherHitComponent = OtherComp;
            mHitPoint = Hit.Location;
            // UPrimitiveComponent*  targetHit =
            // static_cast<UPrimitiveComponent*>(hitActor->GetRootComponent());
            // mPhysicsHandle->GrabComponentAtLocation(targetHit, FName(),
            // Hit.Location);
        }
    }
}

TMap<FString, TTuple<UrdfJointType, UPhysicsConstraintComponent*>>
AUrdfBotPawn::getConstraints()
{
    return this->constraints_;
}

RobotApi* AUrdfBotPawn::getRobotApi() const
{
    RobotApi* resultApi = nullptr;
    ASimModeUrdfBot* urdfSimMode = static_cast<ASimModeUrdfBot*>(simmode_);
    std::vector<std::unique_ptr<RobotSimApi>>& simapiArray =
        urdfSimMode->vehicle_sim_apis_;
    for (auto& ptr : simapiArray)
    {
        if (ptr->getPawn() == this)
        {
            resultApi = ptr->getVehicleApi();
            break;
        }
    }
    return resultApi;
}

void AUrdfBotPawn::setLinkForceAndTorque(FString componentName,
                                         const FVector& force,
                                         const FVector& torque)
{
    // get link
    if (this->components_.Contains(componentName))
    {
        AUrdfLink* botComponentLink = this->components_[componentName];
        UrdfForceSpecification* forceSpec = nullptr;

        // botComponentLink->AddForceSpecification()
    }
}

void AUrdfBotPawn::setJointTorque(FString jointName, const FVector& torque)
{
    // get joint
    if (this->controlled_motion_components_.Contains(jointName))
    {
        ControlledMotionComponent* jointComponent =
            this->controlled_motion_components_[jointName];
        jointComponent->SetConstTorque(
            torque); // jointComponent指针必然不为空，所以不需要做判空处理。

        // FRotator   oritation(torque[0], torque[1], torque[2]);
        // AUrdfLink* parentLink = jointComponent->GetParentLink();
        // AUrdfLink* childLink = jointComponent->GetActuatorLink();
        // UMeshComponent*  parentMesh = parentLink->GetRootMesh();
        // UMeshComponent*  childMesh = childLink->GetRootMesh();
        // parentMesh->GetLinearDamping();
        // UPhysicsConstraintComponent*  phyXcomponent =
        // jointComponent->GetConstraintComponent(); float damping0 =
        // phyXcomponent->ConstraintInstance.ProfileInstance.LinearDrive.XDrive.Damping;
        // float damping1 =
        // phyXcomponent->ConstraintInstance.ProfileInstance.AngularDrive.TwistDrive.Damping;
        // switch (jointComponent->GetMotionType())
        //{
        //	case ControlledMotionComponent::MotionComponentType::MOTOR:
        //		//进行torque的转化处理
        //		phyXcomponent->SetAngularVelocityTarget(torque);
        //		break;
        //	//ToDo other list
        //	case
        // ControlledMotionComponent::MotionComponentType::LINERACTUATOR:
        //		phyXcomponent->SetAngularVelocityTarget(torque);
        //		break;
        //	default:
        //		break;
        //}
    }
}

void AUrdfBotPawn::setDrive(FString jointName, float stiffness, float damping)
{
    if (this->controlled_motion_components_.Contains(jointName))
    {
        ControlledMotionComponent* jointComponent =
            this->controlled_motion_components_[jointName];
        jointComponent->SetDrive(stiffness, damping);
    }
}

void AUrdfBotPawn::setDriveTarget(FString jointName, float target)
{
    if (this->controlled_motion_components_.Contains(jointName))
    {
        ControlledMotionComponent* jointComponent =
            this->controlled_motion_components_[jointName];
        jointComponent->SetDriveTarget(target);
    }
}
USceneComponent* AUrdfBotPawn::GetComponent(FString componentName)
{
    // For debug
    if (!this->components_.Contains(componentName))
    {
        throw std::runtime_error("Requested component named " +
                                 std::string(TCHAR_TO_UTF8(*componentName)) +
                                 " in GetComponent(), which does not exist.");
    }

    return this->components_[componentName]->GetRootComponent();
}

AUrdfLink* AUrdfBotPawn::GetLink(FString linkName)
{
    // For debug
    if (!this->components_.Contains(linkName))
    {
        UE_LOG(LogTemp, Warning,
               TEXT("Requested component named %s in GetComponent(), which "
                    "does not exist. "),
               *linkName);
        // throw std::runtime_error("Requested component named " +
        // std::string(TCHAR_TO_UTF8(*linkName)) + " in GetComponent(), which
        // does not exist.");
        return nullptr;
    }

    return this->components_[linkName];
}

void AUrdfBotPawn::GetComponentReferenceTransform(FString componentName,
                                                  FVector& translation,
                                                  FRotator& rotation)
{
    // For debug
    if (!this->components_.Contains(componentName))
    {
        throw std::runtime_error("Requested component named " +
                                 std::string(TCHAR_TO_UTF8(*componentName)) +
                                 " in GetComponent(), which does not exist.");
    }

    this->components_[componentName]->GetReferenceFrameLocation(translation,
                                                                rotation);
}

void AUrdfBotPawn::ConstructFromFile(FString fileName)
{
    this->components_.Empty();
    this->component_visited_as_constraint_parent_.Empty();
    this->constraints_.Empty();

    this->world_scale_ = URobotBlueprintLib::GetWorldToMetersScale(this);

    UrdfParser parser;
    parser.Parse(fileName);
    TMap<FString, UrdfLinkSpecification*> links = parser.GetLinks();
    TMap<FString, UrdfJointSpecification*> joints = parser.GetJoints();
    TMap<FString, UrdfForceSpecification*> forces = parser.GetForces();

    for (auto kvp : links)
    {
        AUrdfLink* createdLink = this->CreateLinkFromSpecification(*kvp.Value);
        createdLink->linkName_ = kvp.Key;
        this->components_.Add(kvp.Key, createdLink);
        this->component_visited_as_constraint_parent_.Add(kvp.Key, false);
    }

    auto rootLocation = this->GetActorLocation();
    auto rootRotation = this->GetActorRotation();

    UrdfLinkSpecification* rootLinkSpecification =
        this->FindRootNodeSpecification(links);
    this->root_component_ = this->components_[rootLinkSpecification->Name];
    this->root_component_->SetReferenceFrameLocation(this->GetActorLocation(),
                                                     this->GetActorRotation());
    this->root_component_->GetRootComponent()->AttachTo(
        RootComponent, NAME_None, EAttachLocation::KeepRelativeOffset);

    this->SetRootComponent(this->root_component_->GetRootComponent());

    UPrimitiveComponent* rootCollisionComponent =
        this->root_component_->GetCollisionComponent();
    if (rootCollisionComponent != nullptr)
    {
        rootCollisionComponent->SetWorldLocationAndRotation(rootLocation,
                                                            rootRotation);
    }

    UE_LOG(LogTemp, Warning, TEXT("debug!"));
    this->component_visited_as_constraint_parent_[rootLinkSpecification->Name] =
        true;
    for (auto kvp : rootLinkSpecification->Children)
    {
        UrdfLinkSpecification* childLinkSpecification = kvp.Key;
        UrdfJointSpecification* jointSpecification = kvp.Value;
        AUrdfLink* childLink = this->components_[childLinkSpecification->Name];
        this->AttachChildren(this->root_component_, *rootLinkSpecification,
                             childLink, *childLinkSpecification,
                             jointSpecification);
    }

    for (auto kvp : forces)
    {
        UrdfForceSpecification* forceSpecification = kvp.Value;
        AUrdfLink* link = this->components_[kvp.Value->LinkName];

        link->AddForceSpecification(forceSpecification);
    }
}

AUrdfLink* AUrdfBotPawn::CreateLinkFromSpecification(
    const UrdfLinkSpecification& linkSpecification)
{
    UrdfGeometry* visualGeometry = nullptr;
    UrdfGeometry* collisionGeometry = nullptr;

    if (linkSpecification.VisualSpecification != nullptr)
    {
        visualGeometry = linkSpecification.VisualSpecification->Geometry;
        collisionGeometry = linkSpecification.VisualSpecification->Geometry;
    }

    if (linkSpecification.CollisionSpecification != nullptr)
    {
        collisionGeometry = linkSpecification.CollisionSpecification->Geometry;
        if (visualGeometry == nullptr)
        {
            visualGeometry = linkSpecification.CollisionSpecification->Geometry;
        }
    }

    AUrdfLink* link = NewObject<AUrdfLink>(
        this, AUrdfLink::StaticClass(),
        FName(linkSpecification.Name.GetCharArray().GetData()));
    // if (visualGeometry == nullptr || collisionGeometry == nullptr)
    // throw std::runtime_error("Unable to create link " +
    // std::string(TCHAR_TO_UTF8(*linkSpecification.Name)) + ". No visual or
    // geometry node specified.");
    if (visualGeometry == nullptr || collisionGeometry == nullptr)
    {
        return link;
    }

    bool createResult = this->staticMeshGenerator_.CreateUnscaledMeshForLink(
        linkSpecification, visualGeometry, collisionGeometry, this, link,
        this->materials_);
    if (createResult)
    {
        link->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    }
    else
    {
        std::runtime_error("create urdfLink failed");
    }

    this->ResizeLink(link, collisionGeometry);
    if (linkSpecification.InertialSpecification != nullptr)
    {
        link->SetMass(linkSpecification.InertialSpecification->Mass);
    }
    link->SetOwningActor(this);

    return link;
}

void AUrdfBotPawn::AttachChildren(
    AUrdfLink* parentLink,
    const UrdfLinkSpecification& parentLinkSpecification,
    AUrdfLink* childLink,
    const UrdfLinkSpecification& childLinkSpecification,
    UrdfJointSpecification* jointSpecification)
{
    // float Scalefactor = 1.0f;
    float Scalefactor = this->scale_factor_;
    this->component_visited_as_constraint_parent_[parentLink->GetName()] = true;

    // Weld joints
    UPrimitiveComponent* childCollisionComponent =
        childLink->GetCollisionComponent();

    FVector parentReferenceTranslation(0, 0, 0);
    FRotator parentReferenceRotation(0, 0, 0);
    parentLink->GetReferenceFrameLocation(parentReferenceTranslation,
                                          parentReferenceRotation);

    FTransform world2BaseTransform(parentReferenceRotation,
                                   parentReferenceTranslation,
                                   FVector(1., 1., 1.));
    FVector referenceTranslation = world2BaseTransform.TransformPosition(
        jointSpecification->Origin.Origin * Scalefactor * this->world_scale_);
    FRotator referenceRotation =
        world2BaseTransform
            .TransformRotation(
                FMath::RadiansToDegrees(jointSpecification->Origin.RollPitchYaw)
                    .Quaternion())
            .Rotator();
    // FVector referenceTranslation = parentReferenceTranslation +
    // parentReferenceRotation.RotateVector(jointSpecification->Origin.Origin *
    // Scalefactor) * this->world_scale_; FRotator referenceRotation =
    // parentReferenceRotation +
    // FMath::RadiansToDegrees(jointSpecification->Origin.RollPitchYaw);
    childLink->SetReferenceFrameLocation(referenceTranslation,
                                         referenceRotation);

    if (childCollisionComponent != nullptr)
    {
        if (childLinkSpecification.CollisionSpecification != nullptr)
        {
            FVector localPos =
                (jointSpecification->Origin.Origin +
                 childLinkSpecification.CollisionSpecification->Origin.Origin) *
                Scalefactor * this->world_scale_;
            FRotator localOri =
                FMath::RadiansToDegrees(
                    jointSpecification->Origin.RollPitchYaw) +
                FMath::RadiansToDegrees(
                    childLinkSpecification.CollisionSpecification->Origin
                        .RollPitchYaw);

            childCollisionComponent->SetWorldLocationAndRotation(
                world2BaseTransform.TransformPosition(localPos),
                world2BaseTransform.TransformRotation(localOri.Quaternion())
                    .Rotator());
        }
        else
        {
            childCollisionComponent->SetWorldLocationAndRotation(
                referenceTranslation, referenceRotation);
        }
    }
    if (childLinkSpecification.VisualSpecification != nullptr)
    {
        FRotator visualOffsetRotator = FMath::RadiansToDegrees(
            childLinkSpecification.VisualSpecification->Origin.RollPitchYaw);
        FVector visualOffsetVector =
            visualOffsetRotator.RotateVector(
                childLinkSpecification.VisualSpecification->Origin.Origin *
                Scalefactor) *
            this->world_scale_;

        FVector localPos =
            (jointSpecification->Origin.Origin +
             childLinkSpecification.VisualSpecification->Origin.Origin) *
            Scalefactor * this->world_scale_;
        FRotator localOri =
            FMath::RadiansToDegrees(jointSpecification->Origin.RollPitchYaw) +
            FMath::RadiansToDegrees(childLinkSpecification.VisualSpecification
                                        ->Origin.RollPitchYaw);

        childLink->SetActorLocationAndRotation(
            world2BaseTransform.TransformPosition(localPos),
            world2BaseTransform.TransformRotation(localOri.Quaternion())
                .Rotator());
        childLink->RecordVisualOffset(visualOffsetVector, visualOffsetRotator);
    }

    // Create constraint component
    UPhysicsConstraintComponent* constraint =
        NewObject<UPhysicsConstraintComponent>(
            (USceneComponent*)parentLink,
            FName(*(parentLinkSpecification.Name + FString(TEXT("_")) +
                    childLinkSpecification.Name)));
    FConstraintInstance constraintInstance =
        this->CreateConstraint(*jointSpecification);

    constraint->ConstraintInstance = constraintInstance;

    FRotator rotation = FRotator::ZeroRotator;
    if (jointSpecification->Type != FIXED_TYPE)
    {
        // Rotate such that the local X axis aligns with the specified axis
        FVector unitX(1, 0, 0);
        FVector unitY(0, 1, 0);
        FVector unitZ(0, 0, 1);

        FRotator jointAxisRotator(
            FMath::RadiansToDegrees(jointSpecification->RollPitchYaw));
        FVector jointAxisInParentFrame =
            parentReferenceRotation.RotateVector(jointSpecification->Axis);
        FVector jointAxis =
            jointAxisRotator.RotateVector(jointAxisInParentFrame);

        // FVector planeToParentRotAxis = unitX ^ jointAxis;
        // if (planeToParentRotAxis.Normalize())
        //{
        //    float rotRad = acosf(FVector::DotProduct(unitX,
        //    jointAxisInParentFrame)); FQuat rotQuat(planeToParentRotAxis,
        //    rotRad); FRotator rotRot = rotQuat.Rotator(); rotation += rotRot;
        //}

        //// Specal case - jointSpecification->Axis = - X. Then, roll 180
        /// degrees
        // if (jointSpecification->Axis.X == -1 && jointSpecification->Axis.Y ==
        // 0 && jointSpecification->Axis.Z == 0)
        //{
        //    float rotRad = FMath::DegreesToRadians(-180);
        //    FQuat rotQuat(unitX, rotRad);
        //    FRotator rotRot = rotQuat.Rotator();
        //    rotation += rotRot;
        //}

        // Center the joint for revolute joints
        if (jointSpecification->Type == REVOLUTE_TYPE)
        {
            float medianPosition =
                FMath::RadiansToDegrees((jointSpecification->Limit->Upper +
                                         jointSpecification->Limit->Lower) *
                                        0.5f);
            constraint->ConstraintInstance.AngularRotationOffset =
                FRotator(0, 0, medianPosition);
        }

        FRotator rotation1 = FRotationMatrix::MakeFromX(jointAxis).Rotator();
        constraint->SetWorldRotation(rotation1, false, nullptr,
                                     ETeleportType::TeleportPhysics);
    }

    // If required, move child component to set up the physics constraint
    FVector inverseTransform = this->MoveChildLinkForLimitedXAxisMotion(
        parentLink, childLink, *jointSpecification);

    constraint->SetDisableCollision(true);

    constraint->SetWorldLocation(childLink->GetActorLocation());

    constraint->AttachToComponent(
        parentLink->GetRootComponent(),
        FAttachmentTransformRules(EAttachmentRule::KeepWorld,
                                  (jointSpecification->Type == FIXED_TYPE)));
    //将父子link都设置为以physical joint 为父节点。
    // constraint->AttachToComponent(this->GetRootComponent(),
    // FAttachmentTransformRules(EAttachmentRule::KeepWorld,
    // (jointSpecification->Type == FIXED_TYPE)));
    // parentLink->GetRootMesh()->AttachToComponent(constraint,
    // FAttachmentTransformRules(EAttachmentRule::KeepWorld,
    // (jointSpecification->Type == FIXED_TYPE)));
    // childLink->GetRootMesh()->AttachToComponent(constraint,
    // FAttachmentTransformRules(EAttachmentRule::KeepWorld,
    // (jointSpecification->Type == FIXED_TYPE)));

    constraint->ConstraintActor1 = this;
    constraint->ConstraintActor2 = this;

    constraint->SetConstrainedComponents(parentLink->GetRootMesh(), NAME_None,
                                         childLink->GetRootMesh(), NAME_None);

    this->constraints_.Add(jointSpecification->Name,
                           TTuple<UrdfJointType, UPhysicsConstraintComponent*>(
                               jointSpecification->Type, constraint));

    if (this->ConstraintNeedsControlledMotionComponent(*jointSpecification))
    {
        ControlledMotionComponent* component =
            ControlledMotionComponentFactory::CreateControlledMotionComponent(
                parentLink, childLink, jointSpecification, constraint);
        this->controlled_motion_components_.Add(component->GetName(),
                                                component);
        if (jointSpecification->Type == CONTINUOUS_TYPE &&
            jointSpecification->Limit)
        {
            this->movement_components_.Add(component->GetName(), component);
        }
        childLink->parentJointName_ = component->GetName();
    }

    // Reset Child if needed
    FVector cl = childLink->GetActorLocation();
    childLink->SetActorLocation(cl + inverseTransform, false, nullptr,
                                ETeleportType::TeleportPhysics);
    FVector cl2 = childLink->GetActorLocation();

    // Attach all of the child node's children
    if (!this->component_visited_as_constraint_parent_[childLinkSpecification
                                                           .Name])
    {
        for (auto kvp : childLinkSpecification.Children)
        {
            UrdfLinkSpecification* nextChildLinkSpecification = kvp.Key;
            UrdfJointSpecification* nextJointSpecification = kvp.Value;
            AUrdfLink* nextChildLink =
                this->components_[nextChildLinkSpecification->Name];
            this->AttachChildren(childLink, childLinkSpecification,
                                 nextChildLink, *nextChildLinkSpecification,
                                 nextJointSpecification);
        }
    }
}

FConstraintInstance
AUrdfBotPawn::CreateConstraint(const UrdfJointSpecification& jointSpecification)
{
    FConstraintInstance constraintInstance =
        this->CreateDefaultFixedConstraintInstance();

    constraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = false;
    constraintInstance.ProfileInstance.ConeLimit.bSoftConstraint = false;
    constraintInstance.ProfileInstance.AngularDrive.AngularDriveMode =
        EAngularDriveMode::TwistAndSwing;

    float range = 0.0f;
    switch (jointSpecification.Type)
    {
    case FIXED_TYPE:
        break;
    case FLOATING_TYPE:
        constraintInstance.SetAngularTwistLimit(
            EAngularConstraintMotion::ACM_Free, 0.0f);
        constraintInstance.SetAngularSwing1Limit(
            EAngularConstraintMotion::ACM_Free, 0.0f);
        constraintInstance.SetAngularSwing2Limit(
            EAngularConstraintMotion::ACM_Free, 0.0f);
        break;
    case PRISMATIC_TYPE:
        range = (jointSpecification.Limit->Upper -
                 jointSpecification.Limit->Lower) *
                this->world_scale_ * 0.5f * this->scale_factor_;
        constraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited,
                                           range);
        if (jointSpecification.Limit != nullptr &&
            jointSpecification.Limit->Effort > 0)
        {
            constraintInstance.SetLinearDriveParams(
                jointSpecification.Limit->Effort * this->world_scale_, 5.0f,
                jointSpecification.Limit->Effort * this->world_scale_);
            constraintInstance.SetLinearPositionDrive(true, false, false);
        }
        else
        {
            constraintInstance.SetLinearDriveParams(0, 0, 0);
            constraintInstance.SetLinearPositionDrive(false, false, false);
        }
        break;
    case REVOLUTE_TYPE:
        range = FMath::RadiansToDegrees(jointSpecification.Limit->Upper -
                                        jointSpecification.Limit->Lower) *
                0.5f;
        //设置twist 就是将活动限制再Z方向
        constraintInstance.SetAngularTwistLimit(
            EAngularConstraintMotion::ACM_Limited, range);
        if (jointSpecification.Limit != nullptr &&
            jointSpecification.Limit->Effort > 0)
        {
            constraintInstance.SetAngularPositionDrive(false, true);
            constraintInstance.SetAngularDriveParams(
                jointSpecification.Limit->Effort * this->world_scale_, 5.0f,
                jointSpecification.Limit->Effort * this->world_scale_);
        }
        else
        {
            constraintInstance.SetAngularPositionDrive(false, false);
            constraintInstance.SetAngularDriveParams(0, 0, 0);
        }
        break;
    case CONTINUOUS_TYPE:
        constraintInstance.SetAngularTwistLimit(
            EAngularConstraintMotion::ACM_Free, 0.0f);
        constraintInstance.SetAngularSwing1Limit(
            EAngularConstraintMotion::ACM_Locked, 0.0f);
        constraintInstance.SetAngularSwing2Limit(
            EAngularConstraintMotion::ACM_Locked, 0.0f);
        if (jointSpecification.Limit != nullptr &&
            jointSpecification.Limit->Effort > 0)
        {
            constraintInstance.SetAngularPositionDrive(false, true);
            constraintInstance.SetAngularVelocityDrive(false, true);
            // UE_LOG(LogTemp, Warning, TEXT("this->world_scale_: %s "),
            // this->world_scale_);
            constraintInstance.SetAngularDriveParams(
                5.0f,
                jointSpecification.Limit->Effort * this->world_scale_ *
                    this->world_scale_,
                jointSpecification.Limit->Effort * this->world_scale_ *
                    this->world_scale_);
        }
        else
        {
            constraintInstance.SetAngularVelocityDrive(false, false);
            constraintInstance.SetAngularDriveParams(0, 0, 0);
        }
        break;
    case PLANAR_TYPE:
        range = (jointSpecification.Limit->Upper -
                 jointSpecification.Limit->Lower) *
                this->world_scale_ * 0.5f;
        constraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited,
                                           range);
        constraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Limited,
                                           range);
        constraintInstance.SetLinearZLimit(ELinearConstraintMotion::LCM_Locked,
                                           0);
        break;
    }

    return constraintInstance;
}

void AUrdfBotPawn::ResizeLink(AUrdfLink* link, UrdfGeometry* geometry)
{
    FVector resizeSpec = FVector(0, 0, 0);
    UrdfBox* boxSpecification = nullptr;
    UrdfCylinder* cylinderSpecification = nullptr;
    UrdfSphere* sphereSpecification = nullptr;
    switch (geometry->GetGeometryType())
    {
    case BOX:
        boxSpecification = static_cast<UrdfBox*>(geometry);
        resizeSpec = boxSpecification->Size;
        break;
    case CYLINDER:
        cylinderSpecification = static_cast<UrdfCylinder*>(geometry);
        resizeSpec = FVector(cylinderSpecification->Radius * 2.0f,
                             cylinderSpecification->Radius * 2.0f,
                             cylinderSpecification->Length);
        break;
    case SPHERE:
        sphereSpecification = static_cast<UrdfSphere*>(geometry);
        resizeSpec = FVector(sphereSpecification->Radius * 2.0f,
                             sphereSpecification->Radius * 2.0f,
                             sphereSpecification->Radius * 2.0f);
        break;
    case MESH:
        resizeSpec = FVector(1, 1, 1);
        break;
    default:
        throw std::runtime_error("Unable to construct shape component due to "
                                 "unrecognized mesh shape.");
    }
    link->GetRootComponent()->SetWorldScale3D(resizeSpec * this->scale_factor_);
}

UrdfLinkSpecification* AUrdfBotPawn::FindRootNodeSpecification(
    TMap<FString, UrdfLinkSpecification*> links)
{
    for (auto kvp : links)
    {
        UrdfLinkSpecification* candidateLink = kvp.Value;
        if (candidateLink->ParentLink == nullptr)
            return candidateLink;
    }

    throw std::runtime_error("Cannot construct the bot. No root node.");
}

FConstraintInstance AUrdfBotPawn::CreateDefaultFixedConstraintInstance()
{
    FConstraintInstance ConstraintInstance;
    ConstraintInstance.SetDisableCollision(true);
    ConstraintInstance.SetLinearXMotion(ELinearConstraintMotion::LCM_Locked);
    ConstraintInstance.SetLinearYMotion(ELinearConstraintMotion::LCM_Locked);
    ConstraintInstance.SetLinearZMotion(ELinearConstraintMotion::LCM_Locked);
    ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Locked,
                                       0.0f);
    ConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Locked,
                                       0.0f);
    ConstraintInstance.SetLinearZLimit(ELinearConstraintMotion::LCM_Locked,
                                       0.0f);
    ConstraintInstance.SetAngularSwing1Motion(
        EAngularConstraintMotion::ACM_Locked);
    ConstraintInstance.SetAngularSwing2Motion(
        EAngularConstraintMotion::ACM_Locked);
    ConstraintInstance.SetAngularTwistMotion(
        EAngularConstraintMotion::ACM_Locked);
    ConstraintInstance.SetAngularSwing1Limit(
        EAngularConstraintMotion::ACM_Locked, 0.0f);
    ConstraintInstance.SetAngularSwing2Limit(
        EAngularConstraintMotion::ACM_Locked, 0.0f);
    ConstraintInstance.SetAngularTwistLimit(
        EAngularConstraintMotion::ACM_Locked, 0.0f);
    ConstraintInstance.AngularRotationOffset = FRotator(0, 0, 0);

    return ConstraintInstance;
}

bool AUrdfBotPawn::ConstraintNeedsControlledMotionComponent(
    const UrdfJointSpecification& spec)
{
    if (spec.Type == PRISMATIC_TYPE || spec.Type == REVOLUTE_TYPE ||
        spec.Type == CONTINUOUS_TYPE)
    {
        if (spec.Limit != nullptr)
        {
            return (spec.Limit->Effort > 0) &&
                   !(this->controlled_motion_components_.Contains(spec.Name));
        }
        else
        {
            return !(this->controlled_motion_components_.Contains(spec.Name));
        }
    }

    return false;
}

FVector AUrdfBotPawn::MoveChildLinkForLimitedXAxisMotion(
    AUrdfLink* parentLink,
    AUrdfLink* childLink,
    const UrdfJointSpecification& jointSpecification)
{
    FVector inverseTransform = FVector::ZeroVector;

    // This transform is only needed for prismatic joints.
    if (jointSpecification.Type != PRISMATIC_TYPE)
    {
        return inverseTransform;
    }

    // If prismatic joint does not have limit set, then no transform is needed
    if (jointSpecification.Limit == nullptr ||
        jointSpecification.Limit->Lower < 0 - 1 ||
        jointSpecification.Limit->Upper < 0)
    {
        return inverseTransform;
    }

    if (jointSpecification.Limit->Lower >= jointSpecification.Limit->Upper)
    {
        std::string jointName =
            std::string(TCHAR_TO_UTF8(*jointSpecification.Name));
        std::string lower = std::to_string(jointSpecification.Limit->Lower);
        std::string upper = std::to_string(jointSpecification.Limit->Upper);

        throw std::runtime_error("Joint '" + jointName + "' has a lower of '" +
                                 lower + "', which is >= the upper of '" +
                                 upper + "'.");
    }

    FVector parentLocation = parentLink->GetActorLocation();
    FVector childLocation = childLink->GetActorLocation();
    FRotator parentRotation = parentLink->GetActorRotation();
    FRotator childRotation = childLink->GetActorRotation();

    float lowerUU = jointSpecification.Limit->Lower * this->world_scale_ *
                    this->scale_factor_;
    float upperUU = jointSpecification.Limit->Upper * this->world_scale_ *
                    this->scale_factor_;

    // FVector xAxisInParentFrame = parentRotation.UnrotateVector(FVector(1, 0,
    // 0));
    FRotator axisRotator = jointSpecification.Axis.Rotation();
    // FVector unitVectorOfAxis =
    // axisRotator.UnrotateVector(xAxisInParentFrame);

    FVector axisInParent = axisRotator.RotateVector(FVector(1, 0, 0));
    FVector unitVectorOfAxis = parentRotation.RotateVector(axisInParent);

    if (!unitVectorOfAxis.Normalize())
    {
        std::string jointName =
            std::string(TCHAR_TO_UTF8(*jointSpecification.Name));
        throw std::runtime_error("Cannot normalize axis for join '" +
                                 jointName + "'.");
    }

    FVector childMinLocation = parentLocation + (unitVectorOfAxis * lowerUU);
    FVector childMaxLocation = parentLocation + (unitVectorOfAxis * upperUU);
    FVector requiredPositionParent =
        (childMinLocation + childMaxLocation) * 0.5f;
    FVector transform = requiredPositionParent - parentLocation;
    inverseTransform = -1 * transform;

    childLink->SetActorLocation(childLocation + transform, false, nullptr,
                                ETeleportType::TeleportPhysics);

    return inverseTransform;
}

void AUrdfBotPawn::DrawDebug()
{
    UWorld* world = this->GetWorld();

    // FColor jointColor = FColor::Emerald;
    FColor jointColor = FColor::White;
    bool persistant = false;
    float persistSeconds = -1;
    uint8 priority = (uint8)'\002';
    float thickness = 0.5f;
    int32 jointCircleSegments = 32;
    float jointCircleRadius = this->debug_symbol_scale_ / 2.0f;
    float axisLineLength = this->debug_symbol_scale_;

    // Draw debug axis
    for (auto& kvp : this->components_)
    {
        AUrdfLink* component = kvp.Value;

        FVector actualLocation = component->GetActorLocation();
        FRotator actualRotation = component->GetActorRotation();

        FVector xx(axisLineLength, 0, 0);
        FVector yy(0, axisLineLength, 0);
        FVector zz(0, 0, axisLineLength);
        FVector zzz(0, 0, 0);

        FVector xxt = actualRotation.RotateVector(xx) + actualLocation;
        FVector yyt = actualRotation.RotateVector(yy) + actualLocation;
        FVector zzt = actualRotation.RotateVector(zz) + actualLocation;
        FVector zzzt = actualRotation.RotateVector(zzz) + actualLocation;

        DrawDebugLine(world, zzzt, xxt, FColor::Red, persistant, persistSeconds,
                      priority, thickness);
        DrawDebugLine(world, zzzt, yyt, FColor::Green, persistant,
                      persistSeconds, priority, thickness);
        DrawDebugLine(world, zzzt, zzt, FColor::Blue, persistant,
                      persistSeconds, priority, thickness);
    }

    // Draw constraints
    for (auto& kvp : this->constraints_)
    {
        UPhysicsConstraintComponent* component = kvp.Value.Value;
        UrdfJointType jointType = kvp.Value.Key;

        if (jointType == REVOLUTE_TYPE)
        {
            FVector directionLocal(0, jointCircleRadius / 2, 0);
            FRotator componentRotator = component->GetComponentRotation();
            FVector unitX(1, 0, 0);
            FVector localX = componentRotator.RotateVector(unitX);

            float rotationAmount =
                component->ConstraintInstance.GetAngularTwistLimit();
            float center =
                component->ConstraintInstance.AngularRotationOffset.Roll;
            float minRotation = center - rotationAmount;
            float maxRotation = center + rotationAmount;

            FVector directionMin =
                FRotator(0, 0, -minRotation).RotateVector(directionLocal);
            FVector directionMax =
                FRotator(0, 0, -maxRotation).RotateVector(directionLocal);

            FVector minDirectionGlobal =
                componentRotator.RotateVector(directionMin);
            FVector maxDirectionGlobal =
                componentRotator.RotateVector(directionMax);

            DrawDebugLine(world, component->GetComponentLocation(),
                          component->GetComponentLocation() +
                              minDirectionGlobal,
                          FColor::Yellow, persistant, persistSeconds, priority,
                          thickness * 2);
            DrawDebugLine(world, component->GetComponentLocation(),
                          component->GetComponentLocation() +
                              maxDirectionGlobal,
                          FColor::Magenta, persistant, persistSeconds, priority,
                          thickness * 2);

            FVector unitXLocal = componentRotator.RotateVector(unitX);
            FVector start = component->GetComponentLocation();
            FVector end = start + (unitXLocal * jointCircleRadius / 2);

            DrawDebugLine(world, start, end, FColor::Black, persistant,
                          persistSeconds, priority, thickness * 2);
        }
        else if (jointType == CONTINUOUS_TYPE)
        {
            FVector zAxisLocal(0, 0, 1);
            FVector yAxisLocal(0, 1, 0);

            FVector zAxisWorld =
                component->GetComponentRotation().RotateVector(zAxisLocal);
            FVector yAxisWorld =
                component->GetComponentRotation().RotateVector(yAxisLocal);
            DrawDebugCircle(world, component->GetComponentLocation(),
                            jointCircleRadius, jointCircleSegments, jointColor,
                            persistant, persistSeconds, priority, thickness,
                            yAxisWorld, zAxisWorld, false);
        }
        else if (jointType == PRISMATIC_TYPE)
        {
            float extents =
                component->ConstraintInstance.ProfileInstance.LinearLimit.Limit;
            FVector startLocal(-extents, 0, 0);
            FVector endLocal(extents, 0, 0);
            FVector componentLocation = component->GetComponentLocation();
            FRotator componentRotation = component->GetComponentRotation();
            FVector startOffset = componentRotation.RotateVector(startLocal);
            FVector endOffset = componentRotation.RotateVector(endLocal);
            FVector startWorld = componentLocation + startOffset;
            FVector endWorld = componentLocation + endOffset;

            DrawDebugLine(world, startWorld, endWorld, jointColor, persistant,
                          persistSeconds, priority, thickness / 2);
            DrawDebugBox(world, componentLocation, FVector(2.5, 10, 10),
                         componentRotation.Quaternion(), FColor::Red,
                         persistant, persistSeconds, priority, thickness / 2);
            DrawDebugPoint(world, startWorld, thickness * 4, FColor::Yellow,
                           persistant, persistSeconds, priority);
            DrawDebugPoint(world, endWorld, thickness * 4, FColor::Magenta,
                           persistant, persistSeconds, priority);
        }
        else if (jointType == FIXED_TYPE)
        {
            DrawDebugPoint(world, component->GetComponentLocation(),
                           jointCircleRadius, jointColor, persistant,
                           persistSeconds, priority);
        }
        else if (jointType == FLOATING_TYPE)
        {
            DrawDebugSphere(world, component->GetComponentLocation(),
                            jointCircleRadius, jointCircleSegments, jointColor,
                            persistant, persistSeconds, priority, thickness);
        }
        else if (jointType == PLANAR_TYPE)
        {
            float extents =
                component->ConstraintInstance.ProfileInstance.LinearLimit.Limit;
            FVector localBoxExtents(extents, extents, 0.001f);
            FVector boxExtents =
                component->GetComponentRotation().RotateVector(localBoxExtents);
            DrawDebugBox(world, component->GetComponentLocation(), boxExtents,
                         jointColor, persistant, persistSeconds, priority,
                         thickness);
        }
        else
        {
            throw std::runtime_error("Unrecognized joint type in DrawDebug()");
        }
    }
}

void AUrdfBotPawn::MoveAllComponents(FVector translation, FRotator rotation)
{
    for (auto kvp : this->components_)
    {
        AUrdfLink* component = kvp.Value;
        FVector componentLocation = component->GetActorLocation();
        FRotator actorRotation = component->GetActorRotation();

        component->SetActorLocationAndRotation(translation + componentLocation,
                                               rotation + actorRotation);
    }
}

void AUrdfBotPawn::onStiffnessUp()
{
    for (const auto& kvp : this->controlled_motion_components_)
    {
        float stiffness = 0;
        float damping = 0;
        kvp.Value->GetDrive(&stiffness, &damping);
        stiffness = stiffness * 10;
        damping = damping * 10;
        kvp.Value->SetDrive(stiffness, damping);
        URobotBlueprintLib::LogMessage(
            kvp.Key,
            FString("stiffness=") + FString::SanitizeFloat(stiffness) +
                FString(" damping=") + FString::SanitizeFloat(damping),
            LogDebugLevel::Informational, 30);
    }
}

void AUrdfBotPawn::onStiffnessDown()
{
    for (const auto& kvp : this->controlled_motion_components_)
    {
        float stiffness = 0;
        float damping = 0;
        kvp.Value->GetDrive(&stiffness, &damping);
        stiffness = stiffness / 10;
        damping = damping / 10;
        kvp.Value->SetDrive(stiffness, damping);
        URobotBlueprintLib::LogMessage(
            kvp.Key,
            FString("stiffness=") + FString::SanitizeFloat(stiffness) +
                FString(" damping=") + FString::SanitizeFloat(damping),
            LogDebugLevel::Informational, 30);
    }
}

void AUrdfBotPawn::onBaseMove(float value)
{
    if (!this->mLeftWheelJoint || !this->mRightWheelJoint)
    {
        return;
    }
    FVector leftTarget = this->mLeftWheelJoint->GetConstraintComponent()
                             ->ConstraintInstance.ProfileInstance.AngularDrive
                             .AngularVelocityTarget;
    FVector rightTarget = this->mRightWheelJoint->GetConstraintComponent()
                              ->ConstraintInstance.ProfileInstance.AngularDrive
                              .AngularVelocityTarget;

    float averageTargetVelocity = 0.5 * (leftTarget.X + rightTarget.X);
    float averageActualVelocity = 0.5 / (2 * PI) *
                                  (this->mLeftWheelJoint->GetMotionSpeed() +
                                   this->mRightWheelJoint->GetMotionSpeed());

    if (value == 0)
    {
        // disable drive for free motion
        this->mLeftWheelJoint->EnableDrive(false);
        this->mLeftWheelJoint->SetDriveTargetVelocity(0);

        this->mRightWheelJoint->EnableDrive(false);
        this->mRightWheelJoint->SetDriveTargetVelocity(0);
    }
    else
    {
        if (this->mLeftWheelJoint->GetConstraintComponent()
                ->ConstraintInstance.ProfileInstance.AngularDrive
                .IsVelocityDriveEnabled())
        {
            // update based on previous target
            this->mLeftWheelJoint->SetDriveTargetVelocity(value);
            this->mRightWheelJoint->SetDriveTargetVelocity(value);
        }
        else
        {
            // update based on current velocity
            this->mLeftWheelJoint->EnableDrive(true);
            this->mLeftWheelJoint->SetDriveTargetVelocity(value);

            this->mRightWheelJoint->EnableDrive(true);
            this->mRightWheelJoint->SetDriveTargetVelocity(value);
        }
    }
}

void AUrdfBotPawn::onBaseRotate(float value)
{
    if (value != 0 && this->mLeftWheelJoint && this->mRightWheelJoint)
    {
        this->mLeftWheelJoint->EnableDrive(true);
        FVector currentLeftTarget =
            this->mLeftWheelJoint->GetConstraintComponent()
                ->ConstraintInstance.ProfileInstance.AngularDrive
                .AngularVelocityTarget;
        this->mLeftWheelJoint->SetDriveTargetVelocity(currentLeftTarget.X +
                                                      value);

        this->mRightWheelJoint->EnableDrive(true);
        FVector currentRightTarget =
            this->mRightWheelJoint->GetConstraintComponent()
                ->ConstraintInstance.ProfileInstance.AngularDrive
                .AngularVelocityTarget;
        this->mRightWheelJoint->SetDriveTargetVelocity(currentRightTarget.X -
                                                       value);
    }
}

void AUrdfBotPawn::setDriveVelocity(FString jointName, float target)
{
    Motor* jointComponent =
        static_cast<Motor*>(this->controlled_motion_components_[jointName]);
    if (target == 0)
    {
        // stop both wheel
        jointComponent->EnableDrive(false);
        jointComponent->SetDriveTargetVelocity(0);
    }
    else
    {
        jointComponent->EnableDrive(true);
        UPhysicsConstraintComponent* constraintComponent =
            jointComponent->GetConstraintComponent();
        FVector currentTarget =
            constraintComponent->ConstraintInstance.ProfileInstance.AngularDrive
                .AngularVelocityTarget;
        jointComponent->SetDriveTargetVelocity(target);
    }
}

void AUrdfBotPawn::updateVelocity(FString jointName, float delta)
{
    Motor* jointComponent =
        static_cast<Motor*>(this->controlled_motion_components_[jointName]);
    UPhysicsConstraintComponent* constraintComponent =
        jointComponent->GetConstraintComponent();
    if (delta == 0)
    {
        // stop both wheel
        jointComponent->EnableDrive(false);
        jointComponent->SetDriveTargetVelocity(0);
    }
    else
    {
        jointComponent->EnableDrive(true);
        FVector currentTarget =
            constraintComponent->ConstraintInstance.ProfileInstance.AngularDrive
                .AngularVelocityTarget;
        jointComponent->SetDriveTargetVelocity(currentTarget.X + delta);
    }
}

void AUrdfBotPawn::onTorsoUp()
{
    if (this->controlled_motion_components_.Contains("torso_lift_joint"))
    {
        ControlledMotionComponent* jointComponent =
            this->controlled_motion_components_["torso_lift_joint"];
        jointComponent->SetDriveTarget(jointComponent->GetDriveTarget() + 0.1);
    }
}

void AUrdfBotPawn::onTorsoDown()
{
    if (this->controlled_motion_components_.Contains("torso_lift_joint"))
    {
        ControlledMotionComponent* jointComponent =
            this->controlled_motion_components_["torso_lift_joint"];
        jointComponent->SetDriveTarget(jointComponent->GetDriveTarget() - 0.1);
    }
}

void AUrdfBotPawn::setEndEffector(FString endEffectorName)
{
    this->mEndEffectorName = endEffectorName;
    this->mEndEffectorLink = this->GetLink(endEffectorName);
    this->inverseKinematicComponent->SetEndEffector(endEffectorName);
}

void AUrdfBotPawn::onArmXUp()
{
    KeyBoardControl(FVector(0.1, 0.0, 0.0), FVector(0.0, 0.0, 0.0));
}

void AUrdfBotPawn::onArmXDown()
{
    KeyBoardControl(FVector(-0.1, 0.0, 0.0), FVector(0.0, 0.0, 0.0));
}

void AUrdfBotPawn::onArmYUp()
{
    KeyBoardControl(FVector(0.0, 0.1, 0.0), FVector(0.0, 0.0, 0.0));
}

void AUrdfBotPawn::onArmYDown()
{
    KeyBoardControl(FVector(0.0, -0.1, 0.0), FVector(0.0, 0.0, 0.0));
}

void AUrdfBotPawn::onArmZUp()
{
    KeyBoardControl(FVector(0.0, 0.0, 0.1), FVector(0.0, 0.0, 0.0));
}

void AUrdfBotPawn::onArmZDown()
{
    KeyBoardControl(FVector(0.0, 0.0, -0.1), FVector(0.0, 0.0, 0.0));
}

void AUrdfBotPawn::onArmXRotationUp()
{
    KeyBoardControl(FVector(0.0, 0.0, 0.0), FVector(0.1, 0.0, 0.0));
}

void AUrdfBotPawn::onArmXRotationDown()
{
    KeyBoardControl(FVector(0.0, 0.0, 0.0), FVector(-0.1, 0.0, 0.0));
}

void AUrdfBotPawn::onArmYRotationUp()
{
    KeyBoardControl(FVector(0.0, 0.0, 0.0), FVector(0.0, 0.1, 0.0));
}

void AUrdfBotPawn::onArmYRotationDown()
{
    KeyBoardControl(FVector(0.0, 0.0, 0.0), FVector(0.0, -0.1, 0.0));
}

void AUrdfBotPawn::onArmZRotationUp()
{
    KeyBoardControl(FVector(0.0, 0.0, 0.0), FVector(0.0, 0.0, 0.1));
}

void AUrdfBotPawn::onArmZRotationDown()
{
    KeyBoardControl(FVector(0.0, 0.0, 0.0), FVector(0.0, 0.0, -0.1));
}

void AUrdfBotPawn::onArmReset()
{
    TArray<FString> linkList;
    TArray<FString> jointList;
    FindArmChain(&linkList, &jointList);

    TMap<FString, float> map = TMap<FString, float>();
    for (auto joint : jointList)
    {
        map.Add(joint, 0.0);
    }
    this->SetTargetQPos(map);
}

void AUrdfBotPawn::onTest()
{
    TArray<FString> linkList;
    TArray<FString> jointList;

    FindArmChain(&linkList, &jointList);

    for (auto& kvp : this->components_)
    {
        UMeshComponent* mesh = kvp.Value->GetRootMesh();
        UMaterialInterface* material = mesh->GetMaterial(0);
        UPhysicalMaterial* physical = material->GetPhysicalMaterial();
        URobotBlueprintLib::LogMessage(
            "link: " + kvp.Key,
            material->GetName() + " " +
                FString::SanitizeFloat(physical->Friction) + " " +
                FString::SanitizeFloat(physical->StaticFriction),
            LogDebugLevel::Informational, 30);
    }
    for (auto& kvp : this->constraints_)
    {
        URobotBlueprintLib::LogMessage("joint: " + kvp.Key, "",
                                       LogDebugLevel::Informational, 30);
    }
    UMeshComponent* mesh_root = root_component_->GetRootMesh();
    FTransform transform = mesh_root->GetRelativeTransform();

    URobotBlueprintLib::LogMessage(FString("rootMesh transform: \n"),
                                   transform.ToHumanReadableString(),
                                   LogDebugLevel::Informational, 30);
}

void AUrdfBotPawn::onManipulator()
{
    for (auto& manipulatorJoint : this->mManipulatorJointList)
    {
        float val = 1;
        float before = manipulatorJoint->GetDriveTarget();
        if (before <= 0)
        {
            // val = val;
        }
        else
        {
            val = -val;
        }
        manipulatorJoint->SetDriveTarget(val);
    }
}

void AUrdfBotPawn::KeyBoardControl(FVector deltaPos, FVector deltaOri)
{
    TMap<FString, float> Qinit = this->GetTargetQPosByLinkName();
    FTransform currentPose =
        getRelativePoseByLink(this->root_component_, this->mEndEffectorLink);
    FVector target_pos = currentPose.GetTranslation() + deltaPos;
    FQuat target_ori = FQuat::MakeFromEuler(currentPose.GetRotation().Euler() +
                                            100 * deltaOri);
    FTransform temp(target_ori, target_pos, FVector(1.0, 1.0, 1.0));
    URobotBlueprintLib::LogMessage(FString("end effector target"),
                                   *temp.ToHumanReadableString(),
                                   LogDebugLevel::Informational, 30);

    TMap<FString, float> resultMap;
    bool flag = this->inverseKinematicComponent->CalculateIK(
        Qinit, this->mEndEffectorName, target_pos, target_ori, resultMap);
    if (flag)
    {
        SetTargetQPosByLinkName(resultMap);
    }
    else
    {
        URobotBlueprintLib::LogMessage(FString("unreachable position?"),
                                       target_pos.ToString() + FString("\n") +
                                           target_ori.Euler().ToString(),
                                       LogDebugLevel::Failure, 30);
    }
}

FTransform AUrdfBotPawn::getJointPose(const FString& jointName)
{
    ControlledMotionComponent* jointComponent =
        this->controlled_motion_components_[jointName];
    const FTransform& World2Base =
        jointComponent->GetParentLink()->GetTransform();
    FTransform temp(World2Base.GetRotation(), World2Base.GetTranslation(),
                    FVector(1, 1, 1));
    AUrdfLink* childLink = jointComponent->GetActuatorLink();

    FVector parent2childPos =
        temp.InverseTransformPosition(childLink->GetActorLocation());
    FQuat parent2childOri = temp.InverseTransformRotation(
        childLink->GetActorRotation().Quaternion());

    return FTransform(parent2childOri, parent2childPos / this->world_scale_,
                      FVector(1.0, 1.0, 1.0));
}

FTransform AUrdfBotPawn::getRelativePose(const FString& baseLinkName,
                                         const FString& TargetlinkName)
{
    if (this->components_.Contains(baseLinkName) &&
        this->components_.Contains(TargetlinkName))
    {
        return getRelativePoseByLink(this->components_[baseLinkName],
                                     this->components_[TargetlinkName]);
    }
    else
    {
        return FTransform();
    }
}

FTransform AUrdfBotPawn::getRelativePoseByLink(AUrdfLink* baseLink,
                                               AUrdfLink* targetLink)
{
    const FTransform& World2Base = baseLink->GetTransform();
    FTransform temp(World2Base.GetRotation(), World2Base.GetTranslation(),
                    FVector(1, 1, 1));
    FVector base2targetPos =
        temp.InverseTransformPosition(targetLink->GetActorLocation());
    FQuat base2targetOri = temp.InverseTransformRotation(
        targetLink->GetActorRotation().Quaternion());

    return FTransform(base2targetOri, base2targetPos / this->world_scale_,
                      FVector(1.0, 1.0, 1.0));
}

TMap<FString, float> AUrdfBotPawn::GetTargetQPosByLinkName()
{
    TMap<FString, float> map;
    for (const TPair<FString, ControlledMotionComponent*>& pair :
         this->controlled_motion_components_)
    {
        map.Add(pair.Value->GetActuatorLink()->linkName_,
                pair.Value->GetDriveTarget());
    }
    return map;
}

TMap<FString, float> AUrdfBotPawn::GetCurrentQPosByLinkName()
{
    TArray<FString> linkList;
    TArray<FString> jointList;

    FindArmChain(&linkList, &jointList);
    TMap<FString, float> map;
    for (FString str : jointList)
    {
        ControlledMotionComponent* component =
            this->controlled_motion_components_[str];
        FTransform transform = getJointPose(str);
        URobotBlueprintLib::LogMessage(
            str,
            FString::SanitizeFloat(180 / PI * component->GetDriveTarget()) +
                FString(" -> ") + transform.GetRotation().Euler().ToString(),
            LogDebugLevel::Informational, 30);
    }
    for (const TPair<FString, ControlledMotionComponent*>& pair :
         this->controlled_motion_components_)
    {
        map.Add(pair.Key, pair.Value->GetDriveTarget());
    }
    return map;
}

void AUrdfBotPawn::SetTargetQPosByLinkName(const TMap<FString, float>& map)
{
    for (const TPair<FString, float>& pair : map)
    {
        if (this->components_.Contains(pair.Key))
        {
            FString jointName = this->components_[pair.Key]->parentJointName_;
            this->controlled_motion_components_[jointName]->SetDriveTarget(
                pair.Value);
        }
    }
}

void AUrdfBotPawn::SetTargetQPos(const TMap<FString, float>& map)
{
    for (const TPair<FString, float>& pair : map)
    {
        if (this->controlled_motion_components_.Contains(pair.Key))
        {
            ControlledMotionComponent* jointComponent =
                this->controlled_motion_components_[pair.Key];
            jointComponent->SetDriveTarget(pair.Value);
        }
    }
}

void AUrdfBotPawn::FindArmChain(TArray<FString>* linkList,
                                TArray<FString>* jointList)
{
    linkList->Empty();
    jointList->Empty();

    AUrdfLink* currentLink = mEndEffectorLink;
    TMap<FString, FString> child2ParentMap;

    for (auto& kvp : this->constraints_)
    {
        UPhysicsConstraintComponent* component = kvp.Value.Value;
        FString parentLinkName =
            component->ComponentName1.ComponentName.ToString();
        FString childLinkName =
            component->ComponentName2.ComponentName.ToString();

        child2ParentMap.Add(childLinkName.Replace(TEXT("_visual"), TEXT("")),
                            parentLinkName.Replace(TEXT("_visual"), TEXT("")));
    }

    while (currentLink && currentLink != this->root_component_)
    {
        FString parentJointName = currentLink->parentJointName_;
        if (!parentJointName.IsEmpty())
        {
            ControlledMotionComponent* joint =
                this->controlled_motion_components_[parentJointName];
            currentLink = joint->GetParentLink();
            jointList->Add(parentJointName);
            linkList->Add(currentLink->GetName());
        }
        else
        {
            // fixed joint, get parent link from map data
            FString parentLinkName = child2ParentMap[currentLink->GetName()];
            currentLink = this->components_[parentLinkName];
        }
    }
}
