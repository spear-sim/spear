#include "VWDoorManager.h"
#include "EngineUtils.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

TArray<FDoorInfo> UVWDoorManager::level_door_info_;

bool UVWDoorManager::initLevelDoorInfo(UWorld* world)
{    
    //cleanup previous data
    level_door_info_.Empty();
    UDataTable* door_data_table = LoadObject<UDataTable>(nullptr,TEXT("DataTable'/VirtualWorldManager/Koolab/SceneInfo/doors_info.doors_info'"));

    if (world == nullptr) {
        return false;
    }
    FString map_name = world->GetName();
    if (!map_name.StartsWith("Map_")) {
        return false;
    }

    FSceneDoorInfo* info = door_data_table->FindRow<FSceneDoorInfo>(FName(map_name.Mid(4)), "doors");
    if (info == nullptr) {
        return false;
    }
    level_door_info_.Append(info->doors);
    matchDoorActor(world);
    return true;
}

void UVWDoorManager::matchDoorActor(UWorld* world)
{
    for (TActorIterator<AActor> it(world); it; ++it) {
        // TODO use other way to identify door actors
        if (it->GetName().StartsWith("Group_")) {
            for (auto& child : it->GetComponents()) {
                // mark if child name containes door component (INSTid1216)
                if (child->GetName().StartsWith("Architecture_") && child->GetName().Contains("INSTid1216")) {
                    it->Tags.Add("door");
                    FBox doorBBox = it->GetComponentsBoundingBox(false, true);
                    for (auto& doordata : level_door_info_) {
                        // check if is position is in door bbox
                        FVector doorCenter = doordata.position;
                        FVector result = doorBBox.GetClosestPointTo(doorCenter);
                        if (doordata.doorActor == nullptr && doorCenter.Equals(result)) {
                            doordata.doorActor = *it;
                            break;
                        }
                    }
                    break;
                }
            }
        }
    }
    // check if all matcked
    for (auto& doordata : level_door_info_) {
        if (doordata.doorActor == nullptr) {
            UE_LOG(LogTemp, Warning, TEXT("ADoorProcessor::MatchDoorActor missing actor at %s"), *(doordata.position.ToString()));
        }
    }
}

bool UVWDoorManager::moveAllDoor(bool open)
{
    for (auto& doordata : level_door_info_) {
        AActor* DoorActor = doordata.doorActor;
        if (DoorActor == nullptr) {
            continue;
        }
        if (doordata.mode == "counter-clockwise" || doordata.mode == "clockwise") {
            for (auto& child : DoorActor->GetComponentsByClass(UStaticMeshComponent::StaticClass())) {
                // find Animation component
                if (child->GetName().StartsWith("Animation_")) {
                    UStaticMeshComponent* anmiComponent = Cast<UStaticMeshComponent>(child);
                    FTransform anmiTransform = FTransform(anmiComponent->GetRelativeTransform());
                    if (doordata.isInnerDoor) {
                        if (doordata.mode == "counter-clockwise") {
                            if (open) {
                                anmiTransform.SetRotation(FRotator(0, -85, 0).Quaternion());
                            }
                            else {
                                anmiTransform.SetRotation(FRotator(0, 0, 0).Quaternion());
                            }
                        }
                        else if (doordata.mode == "clockwise") {
                            if (open) {
                                anmiTransform.SetRotation(FRotator(0, 85, 0).Quaternion());
                            }
                            else {
                                anmiTransform.SetRotation(FRotator(0, 0, 0).Quaternion());
                            }
                        }
                        anmiComponent->SetRelativeTransform(anmiTransform, false, nullptr, ETeleportType::ResetPhysics);
                    }
                }
            }
        }
        else if (doordata.mode == "sliding") {
            TArray<UStaticMeshComponent*> animations;
            if (DoorActor == nullptr) {
                continue;
            }
            for (auto& child : DoorActor->GetComponentsByClass(UStaticMeshComponent::StaticClass())) {
                // find Animation component
                if (child->GetName().StartsWith("Animation_")) {
                    animations.Add(Cast<UStaticMeshComponent>(child));
                }
            }
            int AnmiSize = animations.Num();
            if (AnmiSize < 2 || AnmiSize > 4) {
                // not sure what to do with unexpected number of sliding door
                UE_LOG(LogTemp, Log, TEXT("ADoorProcessor::MoveDoor: unknown sliding count: %s %s"), *(DoorActor->GetName()), animations.Num());
                continue;
            }
            FBox doorBBox = DoorActor->GetComponentsBoundingBox(false, true);

            UStaticMeshComponent* TargetMax = nullptr;
            float TargetMaxDistance = 0;
            UStaticMeshComponent* TargetMin = nullptr;
            float TargetMinDistance = TNumericLimits<float>::Max();

            for (auto& anmi : animations) {
                // calculate anmi bounding box
                TArray<USceneComponent*> childs;
                anmi->GetChildrenComponents(true, childs);
                FBox anmiBox = FBox(EForceInit::ForceInit);

                for (auto& child : childs) {
                    if (child->IsRegistered() && child->IsCollisionEnabled()) {
                        anmiBox += child->Bounds.GetBox();
                    }
                }
                // find farest and nearest anmi as target
                float distance = FVector::Distance(anmi->GetComponentLocation(), anmiBox.GetCenter());
                if (distance > TargetMaxDistance) {
                    TargetMaxDistance = distance;
                    TargetMax = anmi;
                }
                if (distance < TargetMinDistance) {
                    TargetMinDistance = distance;
                    TargetMin = anmi;
                }
            }
            // move sliding anmi depending on total Anmi number
            if (AnmiSize == 2 || AnmiSize == 3) {
                // only move max door
                FVector OldLocation = TargetMax->GetRelativeLocation() * FVector(1.75f, 1, 1);
                TargetMax->SetRelativeLocation(OldLocation, false, nullptr, ETeleportType::ResetPhysics);
            }
            if (AnmiSize == 4) {
                TargetMax->SetRelativeLocation(TargetMax->GetRelativeLocation() * FVector(1.75f, 1, 1), false, nullptr, ETeleportType::ResetPhysics);
                FVector OldMinLocation = TargetMin->GetRelativeLocation();
                TargetMin->SetRelativeLocation(TargetMax->GetRelativeLocation() * FVector(-0.75f, 1, 1), false, nullptr, ETeleportType::ResetPhysics);
            }
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("ADoorProcessor::MoveDoor: Unknown doordata.mode:%s %s "), *(DoorActor->GetName()), *(doordata.mode));
        }
    }
    return true;
}
