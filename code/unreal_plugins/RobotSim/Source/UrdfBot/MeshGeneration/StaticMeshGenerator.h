#pragma once

// Something is broken with the RAWMESH_API define symbol in RawMesh.h
// This macro is supposed to be defined somewhere, but I have no idea where.
// For now, define it as the empty string
//#define RAWMESH_API

#include "CoreMinimal.h"
#include "Runtime/Engine/Classes/Components/BoxComponent.h"
#include "Runtime/Engine/Classes/Components/SphereComponent.h"
#include "Runtime/Engine/Classes/Components/CapsuleComponent.h"
#include "Runtime/Engine/Classes/Engine/Polys.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Runtime/Engine/Classes/PhysicsEngine/BodySetup.h"
#include "Runtime/Engine/Public/Model.h"
#include "ProceduralMeshComponent.h"

#include "ProceduralMeshFileParser.h"
#include "ProceduralMeshFileType.h"
#include "ProceduralMeshFileUtilities.h"
#include "ProceduralMeshFileParserFactory.h"
#include "../UrdfParser/UrdfGeometry.h"
#include "../UrdfLink.h"

#include "ThirdParty/VHACD/public/VHACD.h"

#include <stdexcept>

class StaticMeshGenerator
{
    public:
        bool Initialize(UStaticMesh* boxTemplateMesh, UStaticMesh* cylinderTemplateMesh, UStaticMesh* sphereTemplateMesh, TMap<FString, UStaticMesh*> unrealMeshes);
        bool CreateUnscaledMeshForLink(const UrdfLinkSpecification &linkSpecification, UrdfGeometry* visualGeometry, UrdfGeometry* collisionGeometry, APawn* outer, AUrdfLink* link, TMap<FString, UMaterialInterface*> materials);

    private:
        void ParseProceduralMeshSpecification(FString fileName, ProceduralMeshFileType fileType, bool reverseNormals, float scaleFactor, ProceduralMeshSpecification& meshSpecification);
        void SetupPhysicsForDefaultComponent(UPrimitiveComponent* component);
     

        // The below functions are mostly copy-pasted from Editor\UnrealEd\Public\BSPOps.cpp, with minor edits.
        // We cannot take a dependency on UnrealEd, as it will not allow us to package our final build.
        // So, we duplicate the necessary functionality into this library.

        enum ENodePlace
        {
            NODE_Back = 0, // Node is in back of parent              -> Bsp[iParent].iBack.
            NODE_Front = 1, // Node is in front of parent             -> Bsp[iParent].iFront.
            NODE_Plane = 2, // Node is coplanar with parent           -> Bsp[iParent].iPlane.
            NODE_Root = 3, // Node is the Bsp root and has no parent -> Bsp[0].
        };

        enum EBspOptimization
        {
            BSP_Lame,
            BSP_Good,
            BSP_Optimal
        };

        struct FBspIndexedPoint
        {
            FBspIndexedPoint(const FVector& InPoint, int32 InIndex)
                : Point(InPoint)
                , Index(InIndex)
            {}

            FVector Point;
            int32 Index;
        };

        struct FBspPointsGridItem
        {
            TArray<FBspIndexedPoint, TInlineAllocator<16>> IndexedPoints;
        };

        struct FBspPointsKey
        {
            int32 X;
            int32 Y;
            int32 Z;

            FBspPointsKey(int32 InX, int32 InY, int32 InZ)
                : X(InX)
                , Y(InY)
                , Z(InZ)
            {}

            friend FORCEINLINE bool operator == (const FBspPointsKey& A, const FBspPointsKey& B)
            {
                return A.X == B.X && A.Y == B.Y && A.Z == B.Z;
            }

            friend FORCEINLINE uint32 GetTypeHash(const FBspPointsKey& Key)
            {
                return HashCombine(static_cast<uint32>(Key.X), HashCombine(static_cast<uint32>(Key.Y), static_cast<uint32>(Key.Z)));
            }
        };

        class FBspPointsGrid
        {
            public:
                FBspPointsGrid(float InGranularity, float InThreshold, int32 InitialSize = 0)
                    : OneOverGranularity(1.0f / InGranularity)
                    , Threshold(InThreshold)
                {
                    check(InThreshold / InGranularity <= 0.5f);
                    Clear(InitialSize);
                }

                void Clear(int32 InitialSize = 0);

                int32 FindOrAddPoint(const FVector& Point, int32 Index, float Threshold);

            private:
                float OneOverGranularity;
                float Threshold;

                typedef TMap<FBspPointsKey, FBspPointsGridItem> FGridMap;
                FGridMap GridMap;

                int32 GetAdjacentIndexIfOverlapping(int32 GridIndex, float GridPos, float GridThreshold);
        };

        class FBspPointsGridContainer
        {
            public:
                FBspPointsGrid* PointsGrid;
                FBspPointsGrid* VectorsGrid;
        };

        UStaticMesh* boxTemplateMesh_;
        UStaticMesh* cylinderTemplateMesh_;
        UStaticMesh* sphereTemplateMesh_;
        TMap<FString, UStaticMesh*> unrealMeshes_;

        void GenerateBspCollisionMesh(ProceduralMeshSpecification& meshSpecification, UModel* model);
        TArray<TArray<FVector>> CreateCollisionVAHCD(TArray<FVector> stlPoints, TArray<uint32> stlIndices, double concavity, unsigned int resolution, unsigned int maxNumVerticiesPerCH, double minVolumePerCh);
        TArray<TArray<FVector>> ReadManualCollision(FString fileListPath);
        void SaveVhacdGeneratedCollision(TArray<TArray<FVector>> meshes, FString outputFolder);
        TArray<TArray<FPoly>> StaticMeshGeneratorBspBuild(UModel* TempModel, TArray<FPoly> ModelPolys, EBspOptimization Opt, int32 Balance, int32 PortalBias, int32 RebuildSimplePolys, int32 iNode);
        TArray<TArray<FPoly>> StaticMeshGeneratorSplitPolyList(UModel* TempModel, TArray<FPoly> ModelPolys, int32 iParent, ENodePlace NodePlace, int32 NumPolys, FPoly **PolyList, EBspOptimization Opt, int32 Balance, int32 PortalBias, int32 RebuildSimplePolys, FBspPointsGridContainer* GridContainer);
        void BspRefresh(UModel* Model, bool NoRemapSurfs);
        void BspBuildBounds(UModel* Model);
        void TagReferencedNodes(UModel *Model, int32 *NodeRef, int32 *PolyRef, int32 iNode);
        void SplitPartitioner(UModel* Model, FPoly** PolyList, FPoly** FrontList, FPoly** BackList, int32 n, int32 nPolys, int32& nFront, int32& nBack, FPoly InfiniteEdPoly, TArray<FPoly*>& AllocatedFPolys);
        void FilterBound(UModel* Model, FBox* ParentBound, int32 iNode, FPoly** PolyList, int32 nPolys, int32 Outside);
        FPoly BuildInfiniteFPoly(UModel* Model, int32 iNode);
        void UpdateBoundWithPolys(FBox& Bound, FPoly** PolyList, int32 nPolys);
        void UpdateConvolutionWithPolys(UModel *Model, int32 iNode, FPoly **PolyList, int32 nPolys);
        void SplitPolyList(UModel *Model, int32 iParent, ENodePlace NodePlace, int32 NumPolys, FPoly **PolyList, EBspOptimization Opt, int32 Balance, int32 PortalBias, int32 RebuildSimplePolys, FBspPointsGridContainer* GridContainer);
        FPoly* FindBestSplit(int32 NumPolys, FPoly** PolyList, EBspOptimization Opt, int32 Balance, int32 InPortalBias);
        int32 BspAddNode(UModel* Model, int32 iParent, ENodePlace NodePlace, uint32 NodeFlags, FPoly* EdPoly, FBspPointsGridContainer* GridContainer);
        int32 BspAddPoint(UModel* Model, FVector* V, FBspPointsGridContainer* GridContainer, bool Exact);
        int32 BspAddVector(UModel* Model, FVector* V, FBspPointsGridContainer* GridContainer, bool Exact);
        int32 AddThing(TArray<FVector>& Vectors, FVector& V, float Thresh, int32 Check);
};
