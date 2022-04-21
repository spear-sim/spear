#pragma once

#include "NavMesh/NavMeshBoundsVolume.h"
#include "NavigationSystem.h"

namespace RobotSim {

    struct NavMeshUtil {
        // estimate area bounded by vertices
        static float FindArea(TArray<FVector>& vertices)
        {
            float area = 0.0f;
            if (vertices.Num() >= 3) {
                FVector start = vertices[0];
                for (int i = 2; i < vertices.Num(); i++) {
                    area += 0.5f * FVector::CrossProduct(vertices[i - 1] - start, vertices[i] - start).Size();
                }
            }
            return area;
        }
        // check all vert on the same plane, might not guarentee flat?
        static bool IsFlat(FVector& center, TArray<FVector>& OutVerts)
        {
            for (auto v : OutVerts) {
                if (center.Z != v.Z) {
                    return false;
                }
            }
            return true;
        }
        // recursively mark neighbor node
        static void AddtoClusterMap(ARecastNavMesh* navMesh,
                                    TMap<NavNodeRef, NavNodeRef>& clusterMap,
                                    NavNodeRef current,
                                    NavNodeRef root)
        {
            if (!clusterMap.Contains(current)) {
                clusterMap.Add(current, root);
                TArray<NavNodeRef> neighbors;
                navMesh->GetPolyNeighbors(current, neighbors);
                for (auto& neighbor : neighbors) {
                    AddtoClusterMap(navMesh, clusterMap, neighbor, root);
                }
            }
        }
        // clusterize all adjacent nodes
        static void Clusterize(ARecastNavMesh* navMesh,
                               TArray<FNavPoly> Polys,
                               TMap<NavNodeRef, TArray<NavNodeRef>>& clusterMap)
        {
            TMap<NavNodeRef, NavNodeRef> reverseMap;
            for (auto& poly : Polys) {
                AddtoClusterMap(navMesh, reverseMap, poly.Ref, poly.Ref);
            }
            for (auto& kvp : reverseMap) {
                if (!clusterMap.Contains(kvp.Value)) {
                    clusterMap.Add(kvp.Value, TArray<NavNodeRef>());
                }
                clusterMap[kvp.Value].Add(kvp.Key);
            }
        }

        // find cluster area, return clusterId with largest area
        static NavNodeRef FindAreaMap(ARecastNavMesh* navMesh,
                                      TMap<NavNodeRef, TArray<NavNodeRef>>& clusterMap,
                                      TMap<NavNodeRef, float>& areaMap,
                                      bool FilterFlat = true)
        {
            areaMap.Empty();
            for (auto& kvp : clusterMap) {
                areaMap.Add(kvp.Key, 0.0f);
                for (auto PolyId : kvp.Value) {
                    FVector center;
                    navMesh->GetPolyCenter(PolyId, center);
                    TArray<FVector> OutVerts;
                    if (navMesh->GetPolyVerts(PolyId, OutVerts) and (!FilterFlat or IsFlat(center, OutVerts))) {
                        float area = FindArea(OutVerts);
                        areaMap[kvp.Key] += area;
                    }
                }
            }
            TPair<NavNodeRef, float> max;
            for (auto& kvp : areaMap) {
                if (kvp.Value > max.Value) {
                    max = kvp;
                }
            }
            return max.Key;
        }

        static void GetRandomPoint(ARecastNavMesh* navMesh,
                                   FVector& spawnLocation,
                                   float heightLimit)
        {
            if (navMesh) {
                int trial = 0;
                while (trial < 10) {
                    FNavLocation navLocation = navMesh->GetRandomPoint();
                    if (heightLimit <= 0.0f or navLocation.Location.Z < heightLimit) {
                        spawnLocation = navLocation.Location;
                        return;
                    }
                    trial++;
                }
            }
        }

        static void GetRandomPointFromLargestCluster(ARecastNavMesh* navMesh,
                                                     FBox box,
                                                     FVector& spawnLocation)
        {
            if (!navMesh) {
                UE_LOG(LogTemp, Warning, TEXT("invalid navMesh for random spawn"));
                return;
            }
            // Get all Poly
            TArray<FNavPoly> Polys;
            navMesh->GetPolysInBox(box, Polys);
            // clusterization by connectivity
            TMap<NavNodeRef, TArray<NavNodeRef>> clusterMap;
            RobotSim::NavMeshUtil::Clusterize(navMesh, Polys, clusterMap);
            // find Largest cluster
            TMap<NavNodeRef, float> areaMap;
            NavNodeRef clusterIdwithMaxArea = RobotSim::NavMeshUtil::FindAreaMap(navMesh, clusterMap, areaMap);
            FVector startLoc;
            navMesh->GetPolyCenter(clusterIdwithMaxArea, startLoc);
            // get random point in the cluster?
            FNavLocation navLocation;
            navMesh->GetRandomReachablePointInRadius(startLoc, 1000, navLocation);
            spawnLocation = navLocation.Location;
        }
    };

}; // namespace RobotSim
