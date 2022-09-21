// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DummyActor.generated.h"

class ARecastNavMesh;
class ANavMeshBoundsVolume;
class ABrush;

struct FBox;

UCLASS()
class NAVMESHDEBUGPROJECT_API ADummyActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADummyActor();
	~ADummyActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	void rebuildNavSystem();
	FBox getWorldBoundingBox(bool alter_height);

	ARecastNavMesh* nav_mesh_;
	ANavMeshBoundsVolume* dummy_navmesh_bound_volume_;
	//ABrush* brush_actor_;
};
