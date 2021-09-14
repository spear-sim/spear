#pragma once

#include "CoreMinimal.h"

#include "UrdfBot/MeshGeneration/ProceduralMeshFileType.h"

enum UrdfGeometryType
{
    BOX = 0,
    SPHERE,
    CYLINDER,
    MESH
};

enum UrdfGeometryDynamicCollisionType
{
    COL_BSP = 0,
    COL_VHACD,
    COL_MANUAL,
    COL_BOX,
    COL_CYLINER,
    COL_SPHERE
};

class UrdfGeometry
{
public:
    virtual UrdfGeometryType GetGeometryType() = 0;
    virtual ~UrdfGeometry(){};
};

class UrdfBox : public UrdfGeometry
{
public:
    FVector Size;

    UrdfGeometryType GetGeometryType() override
    {
        return BOX;
    }
};

class UrdfCylinder : public UrdfGeometry
{
public:
    float Radius;
    float Length;

    UrdfGeometryType GetGeometryType() override
    {
        return CYLINDER;
    }
};

class UrdfSphere : public UrdfGeometry
{
public:
    float Radius;

    UrdfGeometryType GetGeometryType() override
    {
        return SPHERE;
    }
};

class UrdfMesh : public UrdfGeometry
{
public:
    ProceduralMeshFileType FileType;
    FString FileLocation;
    bool ReverseNormals = false;
    float ScaleFactor = 1.0f;
    double VhacdConcavity = 0.1f;
    unsigned int VhacdResolution = 100000;
    unsigned int VhacdMaxNumVerticesPerCh = 64;
    double VhacdMinVolumePerCh = 0.007f;
    FString VhacdOutputFolderPath = FString(TEXT(""));
    UrdfGeometryDynamicCollisionType DynamicCollisionType = COL_BSP;

    UrdfGeometryType GetGeometryType() override
    {
        return MESH;
    }
};