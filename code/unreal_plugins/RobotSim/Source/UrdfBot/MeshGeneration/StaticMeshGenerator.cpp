#include "StaticMeshGenerator.h"

#include "Rendering/PositionVertexBuffer.h"

bool StaticMeshGenerator::Initialize(UStaticMesh* boxTemplateMesh, UStaticMesh* cylinderTemplateMesh, UStaticMesh* sphereTemplateMesh, TMap<FString, UStaticMesh*> unrealMeshes)
{
    this->boxTemplateMesh_ = boxTemplateMesh;
    this->cylinderTemplateMesh_ = cylinderTemplateMesh;
    this->sphereTemplateMesh_ = sphereTemplateMesh;
    this->unrealMeshes_ = unrealMeshes;

    return true;
}

bool StaticMeshGenerator::CreateUnscaledMeshForLink(const UrdfLinkSpecification &linkSpecification, UrdfGeometry* visualGeometry, UrdfGeometry* collisionGeometry, APawn* outer, AUrdfLink* link, TMap<FString, UMaterialInterface*> materials)
{

	FString linkName = linkSpecification.Name;
	FString linkMaterialName = linkSpecification.VisualSpecification->MaterialName;
    FName collisionName = FName((linkName + TEXT("_collision")).GetCharArray().GetData());

    UrdfBox* boxSpecification = nullptr;
    UrdfCylinder* cylinderSpecification = nullptr;
    UrdfSphere* sphereSpecification = nullptr;
    UrdfMesh* meshSpecification = nullptr;
    UBoxComponent* boxComponent = nullptr;
    UCapsuleComponent* capsuleComponent = nullptr;
    USphereComponent* sphereComponent = nullptr;

    ProceduralMeshSpecification proceduralMeshSpecification;

    bool reparseMeshGeometry = false;
    if (visualGeometry->GetGeometryType() == MESH && collisionGeometry->GetGeometryType() == MESH)
    {
        UrdfMesh* visual = static_cast<UrdfMesh*>(visualGeometry);
        UrdfMesh* collision = static_cast<UrdfMesh*>(collisionGeometry);
        reparseMeshGeometry = ((!visual->FileLocation.Equals(collision->FileLocation)) || (visual->FileType != collision->FileType));
    }

    UrdfGeometryType geometryType = visualGeometry->GetGeometryType();
    FName meshName((linkName + TEXT("_visual")).GetCharArray().GetData());
    if (geometryType == BOX || geometryType == CYLINDER || geometryType == SPHERE)
    {
        UStaticMeshComponent* meshComponent = NewObject<UStaticMeshComponent>(outer, meshName);
        switch (geometryType)
        {
            case BOX:
                meshComponent->SetStaticMesh(this->boxTemplateMesh_);
                break;
            case CYLINDER:
                meshComponent->SetStaticMesh(this->cylinderTemplateMesh_);
                break;
            case SPHERE:
                meshComponent->SetStaticMesh(this->sphereTemplateMesh_);
                break;
            default:
                throw std::runtime_error("what?");
        }

        meshComponent->SetSimulatePhysics(true);
        meshComponent->SetCollisionObjectType(ECollisionChannel::ECC_PhysicsBody);
        meshComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
        meshComponent->SetCollisionEnabled(ECollisionEnabled::Type::QueryAndPhysics);
        meshComponent->SetNotifyRigidBodyCollision(true);

        if (linkMaterialName.Len() > 0)
        {
            meshComponent->SetMaterial(0, materials[linkMaterialName]);
        }

        link->SetMeshFromStaticMeshComponent(meshComponent);
    }
    else if (geometryType == MESH)
    {
        meshSpecification = static_cast<UrdfMesh*>(visualGeometry);

        if (meshSpecification->FileType == UNREAL_MESH)
        {
            UStaticMeshComponent* meshComponent = NewObject<UStaticMeshComponent>(outer, meshName);

            if (!this->unrealMeshes_.Contains(meshSpecification->FileLocation))
            {
                throw std::runtime_error("Unable to find static mesh '" + std::string(TCHAR_TO_UTF8(*meshSpecification->FileLocation)) + "' in StaticMeshGenerator.");
            }

            UStaticMesh* staticMesh = this->unrealMeshes_[meshSpecification->FileLocation];
            meshComponent->SetStaticMesh(staticMesh);

            meshComponent->SetSimulatePhysics(true);
            meshComponent->SetCollisionObjectType(ECollisionChannel::ECC_PhysicsBody);
            meshComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
            meshComponent->SetCollisionEnabled(ECollisionEnabled::Type::QueryAndPhysics);
            meshComponent->SetNotifyRigidBodyCollision(true);
            if (linkMaterialName.Len() > 0)
            {
                meshComponent->SetMaterial(0, materials[linkMaterialName]);
            }
            link->SetMeshFromStaticMeshComponent(meshComponent);
        }
        else if(meshSpecification->FileType == STL_ASCII)
        {
            this->ParseProceduralMeshSpecification(meshSpecification->FileLocation, meshSpecification->FileType, meshSpecification->ReverseNormals, meshSpecification->ScaleFactor, proceduralMeshSpecification);

            UProceduralMeshComponent* meshComponent = NewObject<UProceduralMeshComponent>(outer, meshName);
            meshComponent->CreateMeshSection_LinearColor(
                0,
                proceduralMeshSpecification.Verticies,
                proceduralMeshSpecification.Triangles,
                TArray<FVector>(),
                TArray<FVector2D>(),
                TArray<FLinearColor>(),
                TArray<FProcMeshTangent>(),
                false);
            meshComponent->bUseComplexAsSimpleCollision = false;
            meshSpecification = static_cast<UrdfMesh*>(collisionGeometry);

			//Todo 支持更多的碰撞体。
			TArray<FVector> collisionShapeVertices;
			UrdfLinkCollisionSpecification* urdfCollisionSpecification = linkSpecification.CollisionSpecification;

			if (meshSpecification->DynamicCollisionType == COL_BSP)
			{
				if (reparseMeshGeometry)
				{
					this->ParseProceduralMeshSpecification(meshSpecification->FileLocation, meshSpecification->FileType, meshSpecification->ReverseNormals, meshSpecification->ScaleFactor, proceduralMeshSpecification);
				}

				UModel* tempModel = NewObject<UModel>(outer, TEXT("tmp"));
				tempModel->Initialize(nullptr, 1);

				this->GenerateBspCollisionMesh(proceduralMeshSpecification, tempModel);

				UBodySetup* bs = meshComponent->GetBodySetup();
				bs->Modify();
				bs->CreateFromModel(tempModel, false);

				for (int i = 0; i < bs->AggGeom.ConvexElems.Num(); i++)
				{
					meshComponent->AddCollisionConvexMesh(bs->AggGeom.ConvexElems[i].VertexData);
				}
			}
			else if (meshSpecification->DynamicCollisionType == COL_VHACD)
			{
				if (reparseMeshGeometry)
				{
					meshSpecification = static_cast<UrdfMesh*>(collisionGeometry);
					this->ParseProceduralMeshSpecification(meshSpecification->FileLocation, meshSpecification->FileType, meshSpecification->ReverseNormals, meshSpecification->ScaleFactor, proceduralMeshSpecification);
				}

				// type conversion
				TArray<uint32> triangles;
				for (int i = 0; i < proceduralMeshSpecification.Triangles.Num(); i++)
				{
					triangles.Add(proceduralMeshSpecification.Triangles[i]);
				}

				TArray<TArray<FVector>> vhacdCollision = this->CreateCollisionVAHCD(proceduralMeshSpecification.Verticies, triangles, meshSpecification->VhacdConcavity, meshSpecification->VhacdResolution, meshSpecification->VhacdMaxNumVerticesPerCh, meshSpecification->VhacdMinVolumePerCh);

				meshComponent->SetCollisionConvexMeshes(vhacdCollision);
			}
			else if (meshSpecification->DynamicCollisionType == COL_MANUAL)
			{
				TArray<TArray<FVector>> manualCollision = this->ReadManualCollision(meshSpecification->FileLocation);
				meshComponent->SetCollisionConvexMeshes(manualCollision);
			}
			//TODO   添加基本碰撞体支持
			else if (collisionGeometry->GetGeometryType() == UrdfGeometryType::BOX)
			{
				collisionShapeVertices.Empty();
				UrdfBox*  boxGeometry = static_cast<UrdfBox*>(collisionGeometry);
				FRotator visualOffsetRotator = FMath::RadiansToDegrees(urdfCollisionSpecification->Origin.RollPitchYaw);
				FPositionVertexBuffer* VertexBuffer = &(boxTemplateMesh_->RenderData->LODResources[0].VertexBuffers.PositionVertexBuffer);
				if (VertexBuffer)
				{
					for (uint32 index = 0; index < VertexBuffer->GetNumVertices(); index++)
					{
						FVector originPoint = VertexBuffer->VertexPosition(index);
						FVector scalePoint(originPoint.X * boxGeometry->Size.X , originPoint.Y * boxGeometry->Size.Y , originPoint.Z * boxGeometry->Size.Z);
						FVector rotatedVector = visualOffsetRotator.RotateVector(scalePoint);
						collisionShapeVertices.Add(rotatedVector);
					}
				}
				meshComponent->AddCollisionConvexMesh(collisionShapeVertices);
			}
			else if(collisionGeometry->GetGeometryType() == UrdfGeometryType::CYLINDER)
			{
				collisionShapeVertices.Empty();
				UrdfCylinder*  cylinderGeometry = static_cast<UrdfCylinder*>(collisionGeometry);
				FRotator visualOffsetRotator = FMath::RadiansToDegrees(urdfCollisionSpecification->Origin.RollPitchYaw);
				FPositionVertexBuffer* VertexBuffer = &(cylinderTemplateMesh_->RenderData->LODResources[0].VertexBuffers.PositionVertexBuffer);
				if (VertexBuffer)
				{
					for (uint32 index = 0; index < VertexBuffer->GetNumVertices(); index++)
					{
						FVector originPoint = VertexBuffer->VertexPosition(index);
						FVector scalePoint(originPoint.X * cylinderGeometry->Radius * 2.0 , originPoint.Y * cylinderGeometry->Radius * 2.0, originPoint.Z * cylinderGeometry->Length);
						FVector rotatedVector = visualOffsetRotator.RotateVector(scalePoint);
						collisionShapeVertices.Add(rotatedVector);
					}
				}
				meshComponent->AddCollisionConvexMesh(collisionShapeVertices);
			}
			else
			{
				throw std::runtime_error("Unrecognized collision type in StaticMeshGenerator.");
			}

			//ToDO  setting  urdfbot geometry material 
			if (linkMaterialName.Len() > 0)
			{
				meshComponent->SetMaterial(0, materials[linkMaterialName]);
			}
			meshComponent->SetSimulatePhysics(true);
			meshComponent->SetCollisionObjectType(ECollisionChannel::ECC_PhysicsBody);
			meshComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
			meshComponent->SetCollisionEnabled(ECollisionEnabled::Type::QueryAndPhysics);
            meshComponent->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Overlap); // 忽略机器人link之间的碰撞
			meshComponent->SetNotifyRigidBodyCollision(true);


            // 忽视指定link之间的碰撞
            
            // if(linkName == "torso_fixed_link"){
            //     meshComponent->SetSimulatePhysics(true);
            //     meshComponent->SetCollisionObjectType(ECollisionChannel::ECC_WorldStatic);
            //     meshComponent->SetCollisionResponseToAllChannels(ECR_Ignore);
            //     meshComponent->SetCollisionEnabled(ECollisionEnabled::Type::QueryAndPhysics);
            //     meshComponent->SetNotifyRigidBodyCollision(true);
            //     meshComponent->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Overlap);
            // }else if(linkName == "bellows_link"){
            //     meshComponent->SetSimulatePhysics(true);
            //     meshComponent->SetCollisionObjectType(ECollisionChannel::ECC_WorldStatic);
            //     meshComponent->SetCollisionResponseToAllChannels(ECR_Ignore);
            //     meshComponent->SetCollisionEnabled(ECollisionEnabled::Type::QueryAndPhysics);
            //     meshComponent->SetNotifyRigidBodyCollision(true);
            //     meshComponent->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Overlap);
            // }
            // else if(linkName == "torso_fixed_link"){
            //     meshComponent->SetSimulatePhysics(true);
            //     meshComponent->SetCollisionObjectType(ECollisionChannel::ECC_WorldStatic);
            //     meshComponent->SetCollisionResponseToAllChannels(ECR_Ignore);
            //     meshComponent->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
            //     meshComponent->SetNotifyRigidBodyCollision(true);
            //     meshComponent->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Overlap);
            // }
            // else{
            //     meshComponent->SetSimulatePhysics(true);
			//     meshComponent->SetCollisionObjectType(ECollisionChannel::ECC_PhysicsBody);
			//     meshComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
			//     meshComponent->SetCollisionEnabled(ECollisionEnabled::Type::QueryAndPhysics);
            //     meshComponent->SetCollisionResponseToAllChannels(ECR_Ignore);
            //     meshComponent->SetCollisionResponseToChannel(ECC_WorldStatic, ECR_Overlap);
			//     meshComponent->SetNotifyRigidBodyCollision(true);
            // }

			link->SetMeshFromProceduralMeshComponent(meshComponent);
        }
		else
		{
			throw std::runtime_error("Unrecognized Geomtry FileType in StaticMeshGenerator.");
		}
    }

    return true;
}

TArray<TArray<FVector>> StaticMeshGenerator::CreateCollisionVAHCD(TArray<FVector> stlPoints, TArray<uint32> stlIndices, double concavity, unsigned int resolution, unsigned int maxNumVerticiesPerCH, double minVolumePerCh)
{
    VHACD::IVHACD* vhacdInterface = VHACD::CreateVHACD();

    VHACD::IVHACD::Parameters VHACD_Params;
    VHACD_Params.m_concavity = concavity; // Maximum allowed concavity (default=0.0025, range=0.0-1.0)
    VHACD_Params.m_resolution = resolution; // Maximum number of voxels generated during the voxelization stage (default=100,000, range=10,000-16,000,000)
    VHACD_Params.m_maxNumVerticesPerCH = maxNumVerticiesPerCH; // Controls the maximum number of triangles per convex-hull (default=64, range=4-1024)
    VHACD_Params.m_oclAcceleration = false;
    VHACD_Params.m_minVolumePerCH = minVolumePerCh; // this should be around 1 / (3 * m_resolution ^ (1/3))

    double* verts = new double[3 * stlPoints.Num()];
    double* vertPtr = verts;
    for (int i = 0; i < stlPoints.Num(); i++)
    {
        FVector currentPoint = stlPoints[i];
        *vertPtr = (double)currentPoint.X;
        vertPtr++;
        *vertPtr = (double)currentPoint.Y;
        vertPtr++;
        *vertPtr = (double)currentPoint.Z;
        vertPtr++;
    }

    //const double* const verts = (double*)stlPoints.GetData();
    const unsigned int numVerts = stlPoints.Num();
    const unsigned int* const tris = (unsigned int*)stlIndices.GetData();
    const unsigned int numTris = stlIndices.Num() / 3;
    
    bool success = vhacdInterface->Compute(verts, numVerts, tris, numTris, VHACD_Params);
    //bool success = vhacdInterface->Compute(verts, 3, numVerts, tris, 3, numTris, VHACD_Params);

    if (!success)
    {
        vhacdInterface->Clean();
        vhacdInterface->Release();
        delete[] verts;
        return TArray<TArray<FVector>>();
    }

    int32 numHulls = vhacdInterface->GetNConvexHulls();

    TArray<TArray<FVector>> output;
    for (int32 hullIdx = 0; hullIdx < numHulls; hullIdx++)
    {
        TArray<FVector> thisHull;
        VHACD::IVHACD::ConvexHull hull;
        vhacdInterface->GetConvexHull(hullIdx, hull);

        for (uint32 vertIdx = 0; vertIdx < hull.m_nPoints; vertIdx++)
        {
            FVector V;

            V.X = (float)(hull.m_points[(vertIdx * 3) + 0]);
            V.Y = (float)(hull.m_points[(vertIdx * 3) + 1]);
            V.Z = (float)(hull.m_points[(vertIdx * 3) + 2]);

            thisHull.Add(V);
        }

        output.Add(thisHull);
    }

    vhacdInterface->Clean();
    vhacdInterface->Release();
    delete[] verts;

    return output;
}

TArray<TArray<FVector>> StaticMeshGenerator::ReadManualCollision(FString fileListPath)
{
    TArray<TArray<FVector>> output;
    IFileHandle* rootFile = nullptr;
    
    try
    {
        rootFile = ProceduralMeshFileUtilities::OpenFile(fileListPath, true);

        FString collisionFilePath;
        while (ProceduralMeshFileUtilities::ReadNextLineTrimmingLeadingWhitespace(rootFile, collisionFilePath, 256) || collisionFilePath.Len() > 0)
        {
            TArray<FVector> points;
            IFileHandle* collisionFile = nullptr;

            try
            {
                collisionFile = ProceduralMeshFileUtilities::OpenFile(collisionFilePath, true);

                FString line;
                while (ProceduralMeshFileUtilities::ReadNextLineTrimmingLeadingWhitespace(collisionFile, line, 128) || line.Len() > 0)
                {
                    TArray<FString> splitString;
                    line.ParseIntoArray(splitString, TEXT(" "), true);
					FVector collisionPoint;
					if (splitString.Num() == 0)
					{
						collisionPoint = FVector(0.0, 0.0, 0.0);
					}
                    else if (splitString.Num() == 3)
                    {
						collisionPoint = FVector(FCString::Atof(*splitString[0]), FCString::Atof(*splitString[1]), FCString::Atof(*splitString[2]));
                    }
					else
					{
						throw std::runtime_error("Error parsing 3-d vector from string " + std::string(TCHAR_TO_UTF8(*line)) + ", did not detect 3 elements.");
					}
                    points.Add(collisionPoint);
                }


                delete collisionFile;
                collisionFile = nullptr;

                output.Add(points);
            }
            catch (std::exception &e)
            {
                if (collisionFile != nullptr)
                {
                    delete collisionFile;
                }

                throw e;
            }
        }

        delete rootFile;
        rootFile = nullptr;
    }
    catch (std::exception &e)
    {
        if (rootFile != nullptr)
        {
            delete rootFile;
        }

        throw e;
    }

    return output;

}

void StaticMeshGenerator::SaveVhacdGeneratedCollision(TArray<TArray<FVector>> meshes, FString outputFolder)
{
    IFileHandle* headerFile = nullptr;
    outputFolder = outputFolder.Replace(TEXT("\\"), TEXT("/"));
    FString headerOutputPath = ProceduralMeshFileUtilities::DeepCopyFString(outputFolder);
    FString headerFileName = TEXT("header.txt");
    headerOutputPath.PathAppend(*headerFileName, headerFileName.Len());

    try
    {
        headerFile = ProceduralMeshFileUtilities::OpenFile(headerOutputPath, false);

        for (int meshNum = 0; meshNum < meshes.Num(); meshNum++)
        {
            IFileHandle* meshFile = nullptr;
            FString meshFileName = TEXT("collision_") + FString::FromInt(meshNum) + ".txt";
            FString meshOutputPath = ProceduralMeshFileUtilities::DeepCopyFString(outputFolder);
            meshOutputPath.PathAppend(*meshFileName, meshFileName.Len());
            TArray<FVector> currentMesh = meshes[meshNum];
            //constexpr int byteBufferSize = 4096;
            //uint8 byteBuffer[byteBufferSize];

            try
            {
                meshFile = ProceduralMeshFileUtilities::OpenFile(meshOutputPath, false);

                for (int i = 0; i < currentMesh.Num(); i++)
                {
                    FString outputString;
                    FVector currentPoint = currentMesh[i];
                    outputString += FString::SanitizeFloat(currentPoint.X);
                    outputString += TEXT(" ");
                    outputString += FString::SanitizeFloat(currentPoint.Y);
                    outputString += TEXT(" ");
                    outputString += FString::SanitizeFloat(currentPoint.Z);
                    outputString += LINE_TERMINATOR;
                    meshFile->Write((const uint8*)TCHAR_TO_UTF8(*outputString), outputString.Len());
                }

                meshFile->Flush();
                delete meshFile;
                meshFile = nullptr;

                FString fileNameLine;
                fileNameLine += meshOutputPath;
                fileNameLine += LINE_TERMINATOR;
                headerFile->Write((const uint8*)TCHAR_TO_UTF8(*fileNameLine), fileNameLine.Len());
            }
            catch (std::exception& e)
            {
                if (meshFile != nullptr)
                {
                    delete meshFile;
                }

                throw e;
            }
        }

        headerFile->Flush();
        delete headerFile;
        headerFile = nullptr;
    }
    catch (std::exception& e)
    {
        if (headerFile != nullptr)
        {
            delete headerFile;
        }

        throw e;
    }
}

void StaticMeshGenerator::ParseProceduralMeshSpecification(FString fileName, ProceduralMeshFileType fileType, bool reverseNormals, float scaleFactor, ProceduralMeshSpecification& meshSpecification)
{
    ProceduralMeshFileParserFactory parserFactory;
    TSharedPtr<ProceduralMeshFileParser> meshFileParser = parserFactory.Create(fileType);

    meshSpecification = meshFileParser.Get()->ParseFromFile(fileName);

    if (scaleFactor != 1.0f)
    {
        for (int i = 0; i < meshSpecification.Verticies.Num(); i++)
        {
            meshSpecification.Verticies[i] *= scaleFactor;
        }
    }

    // Some packages (e.g. Blender) spits out the verts in the opposite order that Unreal expects.
    // This means that the normal is facing the wrong way, and we will be looking at the back side of each of the faces.
    if (reverseNormals)
    {
        for (int i = 0; i < meshSpecification.Triangles.Num(); i = i + 3)
        {
            std::swap(meshSpecification.Triangles[i], meshSpecification.Triangles[i + 2]);
        }
    }
}

void StaticMeshGenerator::SetupPhysicsForDefaultComponent(UPrimitiveComponent* component)
{
    if (component != nullptr)
    {
        component->SetSimulatePhysics(true);
        component->SetCollisionObjectType(ECollisionChannel::ECC_PhysicsBody);
        component->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
        component->SetCollisionEnabled(ECollisionEnabled::Type::QueryAndPhysics);
    }
}

void StaticMeshGenerator::GenerateBspCollisionMesh(ProceduralMeshSpecification& meshSpecification, UModel* model)
{
    model->Initialize(nullptr, 1);

    TArray<FPoly> polys;

    int dropCount = 0;
    for (int i = 0; i < meshSpecification.Triangles.Num(); i = i + 3)
    {
        // STL allows for polygons to be specified from three collinear points. 
        // This will cause the BSP build function to return garbage.
        // Because these polygons have no volume, we can safely ignore them.
        //
        // To detect collinearity, we use the fact that give three collinear points, A, B, and C,
        // (A->B) X (A->C) = 0 if the points are collinear.
        //
        // Also: the offsets of these three vectors into the Triangles array is important - don't change them!
        // Otherwise, the collision mesh won't render.
        FVector first = meshSpecification.Verticies[meshSpecification.Triangles[i + 2]];
        FVector second = meshSpecification.Verticies[meshSpecification.Triangles[i + 1]];
        FVector third = meshSpecification.Verticies[meshSpecification.Triangles[i + 0]];

        FVector checkVec = (third - first) ^ (second - first);

        if (checkVec.SizeSquared() >= (float)THRESH_ZERO_NORM_SQUARED)
        {
            auto polygon = new (polys) FPoly();
            new(polygon->Vertices) FVector(first);
            new(polygon->Vertices) FVector(second);
            new(polygon->Vertices) FVector(third);

            polygon->iLink = i / 3;
            int success = polygon->CalcNormal(false);
            if (success != 0)
            {
                std::string message = "Error: corrupt st. file. Computation of normals failed. Vertices are:\n";
                message += "<" + std::to_string(first.X) + ", " + std::to_string(first.Y) + ", " + std::to_string(first.Z) + ">\n";
                message += "<" + std::to_string(second.X) + ", " + std::to_string(second.Y) + ", " + std::to_string(second.Z) + ">\n";
                message += "<" + std::to_string(third.X) + ", " + std::to_string(third.Y) + ", " + std::to_string(third.Z) + ">\n";
                throw std::runtime_error(message);
            }
        }
        else
        {
            dropCount++;
        }
    }

    //model->BuildBound();
    if (polys.Num() > 0)
    {
        TArray<FVector> NewPoints;
        for (int32 i = 0; i < polys.Num(); i++)
            for (int32 j = 0; j< polys[i].Vertices.Num(); j++)
                NewPoints.Add(polys[i].Vertices[j]);
        model->Bounds = FBoxSphereBounds(NewPoints.GetData(), NewPoints.Num());
    }

    this->StaticMeshGeneratorBspBuild(model, polys, BSP_Optimal, 15, 70, 1, 0);
    this->BspRefresh(model, true);
    this->BspBuildBounds(model);
}

TArray<TArray<FPoly>> StaticMeshGenerator::StaticMeshGeneratorBspBuild(UModel* TempModel, TArray<FPoly> ModelPolys, EBspOptimization Opt, int32 Balance, int32 PortalBias, int32 RebuildSimplePolys, int32 iNode)
{
    // TODO: should this really go on the stack?
    // Answer: No. No, this shouldn't. For large meshes, this can cause a stack overflow.
    FBspPointsGrid pointsGrid(50.0f, THRESH_POINTS_ARE_SAME);
    FBspPointsGrid vectorsGrid(1.0f / 16.0f, FMath::Max(THRESH_NORMALS_ARE_SAME, THRESH_VECTORS_ARE_NEAR));

    FBspPointsGridContainer gridContainer;
    gridContainer.PointsGrid = &pointsGrid;
    gridContainer.VectorsGrid = &vectorsGrid;

    int32 OriginalPolys = ModelPolys.Num();
    TArray<TArray<FPoly>> splitPolys = TArray<TArray<FPoly>>();

    if (OriginalPolys)
    {
        // Allocate polygon pool.
        FMemMark Mark(FMemStack::Get());
        FPoly** PolyList = new(FMemStack::Get(), OriginalPolys)FPoly*;

        // Add all FPolys to active list.
        for (int32 i = 0; i < OriginalPolys; i++)
            if (ModelPolys[i].Vertices.Num())
                PolyList[i] = &ModelPolys[i];

        // Now split the entire Bsp by splitting the list of all polygons.
        splitPolys = StaticMeshGeneratorSplitPolyList
        (
            TempModel,
            ModelPolys,
            INDEX_NONE,
            NODE_Root,
            ModelPolys.Num(),
            PolyList,
            Opt,
            Balance,
            PortalBias,
            RebuildSimplePolys,
            &gridContainer
        );

        Mark.Pop();
    }

    return splitPolys;
}

TArray<TArray<FPoly>> StaticMeshGenerator::StaticMeshGeneratorSplitPolyList(UModel* TempModel, TArray<FPoly> ModelPolys, int32 iParent, ENodePlace NodePlace, int32 NumPolys, FPoly **PolyList, EBspOptimization Opt, int32 Balance, int32 PortalBias, int32 RebuildSimplePolys, FBspPointsGridContainer* GridContainer)
{
    FMemMark Mark(FMemStack::Get());

    // Keeping track of allocated FPoly structures to delete later on.
    TArray<FPoly*> AllocatedFPolys;

    // To account for big EdPolys split up.
    int32 NumPolysToAlloc = NumPolys + 8 + NumPolys / 4;
    int32 NumFront = 0; FPoly **FrontList = new(FMemStack::Get(), NumPolysToAlloc)FPoly*;
    int32 NumBack = 0; FPoly **BackList = new(FMemStack::Get(), NumPolysToAlloc)FPoly*;

    FPoly *SplitPoly = FindBestSplit(NumPolys, PolyList, Opt, Balance, PortalBias);

    // Add the splitter poly to the Bsp with either a new BspSurf or an existing one.
    if (RebuildSimplePolys)
    {
        SplitPoly->iLinkSurf = /*Model->Surfs.Num();*/0;
    }

    int32 iOurNode = BspAddNode(TempModel, iParent, NodePlace, 0, SplitPoly, GridContainer);
    int32 iPlaneNode = iOurNode;

    // Now divide all polygons in the pool into (A^2 polygons that are
    // in front of Poly, and (B) polygons that are in back of Poly.
    // Coplanar polys are inserted immediately, before recursing.

    // If any polygons are split by Poly, we ignrore the original poly,
    // split it into two polys, and add two new polys to the pool.
    FPoly *FrontEdPoly = new FPoly;
    FPoly *BackEdPoly = new FPoly;
    // Keep track of allocations.
    AllocatedFPolys.Add(FrontEdPoly);
    AllocatedFPolys.Add(BackEdPoly);

    for (int32 i = 0; i < NumPolys; i++)
    {
        FPoly *EdPoly = PolyList[i];
        if (EdPoly == SplitPoly)
        {
            continue;
        }

        int x = SplitPoly->Vertices[0].X;
        int y = SplitPoly->Vertices[0].Y;
        int z = SplitPoly->Vertices[0].Z;

        switch (EdPoly->SplitWithPlane(SplitPoly->Vertices[0], SplitPoly->Normal, FrontEdPoly, BackEdPoly, 1))
        {
        case SP_Coplanar:
            if (RebuildSimplePolys)
            {
                EdPoly->iLinkSurf = TempModel->Surfs.Num() - 1;
            }
            iPlaneNode = BspAddNode(TempModel, iPlaneNode, NODE_Plane, 0, EdPoly, GridContainer);
            break;

        case SP_Front:
            FrontList[NumFront++] = PolyList[i];
            break;

        case SP_Back:
            BackList[NumBack++] = PolyList[i];
            break;

        case SP_Split:

            // Create front & back nodes.
            FrontList[NumFront++] = FrontEdPoly;
            BackList[NumBack++] = BackEdPoly;

            FrontEdPoly = new FPoly;
            BackEdPoly = new FPoly;
            // Keep track of allocations.
            AllocatedFPolys.Add(FrontEdPoly);
            AllocatedFPolys.Add(BackEdPoly);

            break;
        }
    }

    // Recursively split the front and back pools.
    if (NumFront > 0) StaticMeshGeneratorSplitPolyList(TempModel, ModelPolys, iOurNode, NODE_Front, NumFront, FrontList, Opt, Balance, PortalBias, RebuildSimplePolys, GridContainer);
    if (NumBack > 0) StaticMeshGeneratorSplitPolyList(TempModel, ModelPolys, iOurNode, NODE_Back, NumBack, BackList, Opt, Balance, PortalBias, RebuildSimplePolys, GridContainer);

    // Delete FPolys allocated above. We cannot use FMemStack::Get() for FPoly as the array data FPoly contains will be allocated in regular memory.
    for (int32 i = 0; i < AllocatedFPolys.Num(); i++)
    {
        FPoly* AllocatedFPoly = AllocatedFPolys[i];
        delete AllocatedFPoly;
    }

    Mark.Pop();
    return TArray<TArray<FPoly>>();
}

void StaticMeshGenerator::BspRefresh(UModel* Model, bool NoRemapSurfs)
{
    FMemStack& MemStack = FMemStack::Get();

    FMemMark Mark(MemStack);

    int32 NumNodes = Model->Nodes.Num();
    int32 NumSurfs = Model->Surfs.Num();
    int32 NumVectors = Model->Vectors.Num();
    int32 NumPoints = Model->Points.Num();

    // Remove unreferenced Bsp surfs.
    int32* PolyRef;
    if (NoRemapSurfs)
    {
        PolyRef = NewZeroed<int32>(MemStack, NumSurfs);
    }
    else
    {
        PolyRef = NewOned<int32>(MemStack, NumSurfs);
    }

    int32* NodeRef = NewOned<int32>(MemStack, NumNodes);
    if (NumNodes > 0)
    {
        TagReferencedNodes(Model, NodeRef, PolyRef, 0);
    }

    // Remap Bsp surfs.
    {
        int32 n = 0;
        for (int32 i = 0; i < NumSurfs; i++)
        {
            if (PolyRef[i] != INDEX_NONE)
            {
                Model->Surfs[n] = Model->Surfs[i];
                PolyRef[i] = n++;
            }
        }
        //UE_LOG(LogBSPOps, Log,  TEXT("Polys: %i -> %i"), NumSurfs, n );
        Model->Surfs.RemoveAt(n, NumSurfs - n);
        NumSurfs = n;
    }

    // Remap Bsp nodes.
    {
        int32 n = 0;
        for (int32 i = 0; i < NumNodes; i++)
        {
            if (NodeRef[i] != INDEX_NONE)
            {
                Model->Nodes[n] = Model->Nodes[i];
                NodeRef[i] = n++;
            }
        }
        //UE_LOG(LogBSPOps, Log,  TEXT("Nodes: %i -> %i"), NumNodes, n );
        Model->Nodes.RemoveAt(n, NumNodes - n);
        NumNodes = n;
    }

    // Update Bsp nodes.
    for (int32 i = 0; i < NumNodes; i++)
    {
        FBspNode *Node = &Model->Nodes[i];
        Node->iSurf = PolyRef[Node->iSurf];
        if (Node->iFront != INDEX_NONE) Node->iFront = NodeRef[Node->iFront];
        if (Node->iBack != INDEX_NONE) Node->iBack = NodeRef[Node->iBack];
        if (Node->iPlane != INDEX_NONE) Node->iPlane = NodeRef[Node->iPlane];
    }

    // Remove unreferenced points and vectors.
    int32* VectorRef = NewOned<int32>(MemStack, NumVectors);
    int32* PointRef = NewOned<int32>(MemStack, NumPoints);

    // Check Bsp surfs.
    TArray<int32*> VertexRef;
    for (int32 i = 0; i < NumSurfs; i++)
    {
        FBspSurf *Surf = &Model->Surfs[i];
        VectorRef[Surf->vNormal] = 0;
        VectorRef[Surf->vTextureU] = 0;
        VectorRef[Surf->vTextureV] = 0;
        PointRef[Surf->pBase] = 0;
    }

    // Check Bsp nodes.
    for (int32 i = 0; i < NumNodes; i++)
    {
        // Tag all points used by nodes.
        FBspNode*	Node = &Model->Nodes[i];
        FVert*		VertPool = &Model->Verts[Node->iVertPool];
        for (int B = 0; B < Node->NumVertices; B++)
        {
            PointRef[VertPool->pVertex] = 0;
            VertPool++;
        }
    }

    // Remap points.
    {
        int32 n = 0;
        for (int32 i = 0; i < NumPoints; i++) if (PointRef[i] != INDEX_NONE)
        {
            Model->Points[n] = Model->Points[i];
            PointRef[i] = n++;
        }
        //UE_LOG(LogBSPOps, Log,  TEXT("Points: %i -> %i"), NumPoints, n );
        Model->Points.RemoveAt(n, NumPoints - n);
        NumPoints = n;
    }

    // Remap vectors.
    {
        int32 n = 0;
        for (int32 i = 0; i < NumVectors; i++) if (VectorRef[i] != INDEX_NONE)
        {
            Model->Vectors[n] = Model->Vectors[i];
            VectorRef[i] = n++;
        }
        //UE_LOG(LogBSPOps, Log,  TEXT("Vectors: %i -> %i"), NumVectors, n );
        Model->Vectors.RemoveAt(n, NumVectors - n);
        NumVectors = n;
    }

    // Update Bsp surfs.
    for (int32 i = 0; i < NumSurfs; i++)
    {
        FBspSurf *Surf = &Model->Surfs[i];
        Surf->vNormal = VectorRef[Surf->vNormal];
        Surf->vTextureU = VectorRef[Surf->vTextureU];
        Surf->vTextureV = VectorRef[Surf->vTextureV];
        Surf->pBase = PointRef[Surf->pBase];
    }

    // Update Bsp nodes.
    for (int32 i = 0; i < NumNodes; i++)
    {
        FBspNode*	Node = &Model->Nodes[i];
        FVert*		VertPool = &Model->Verts[Node->iVertPool];
        for (int B = 0; B < Node->NumVertices; B++)
        {
            VertPool->pVertex = PointRef[VertPool->pVertex];
            VertPool++;
        }
    }

    // Shrink the objects.
    //Model->ShrinkModel();

    Model->Vectors.Shrink();
    Model->Points.Shrink();
    Model->Verts.Shrink();
    Model->Nodes.Shrink();
    Model->Surfs.Shrink();
    //Model->LeafHulls.Shrink();

    Mark.Pop();
}

void StaticMeshGenerator::BspBuildBounds(UModel* Model)
{
    if (Model->Nodes.Num() == 0)
        return;

    FPoly Polys[6], *PolyList[6];
    for (int32 i = 0; i < 6; i++)
    {
        PolyList[i] = &Polys[i];
        PolyList[i]->Init();
        PolyList[i]->iBrushPoly = INDEX_NONE;
    }

    new(Polys[0].Vertices)FVector(-HALF_WORLD_MAX, -HALF_WORLD_MAX, HALF_WORLD_MAX);
    new(Polys[0].Vertices)FVector(HALF_WORLD_MAX, -HALF_WORLD_MAX, HALF_WORLD_MAX);
    new(Polys[0].Vertices)FVector(HALF_WORLD_MAX, HALF_WORLD_MAX, HALF_WORLD_MAX);
    new(Polys[0].Vertices)FVector(-HALF_WORLD_MAX, HALF_WORLD_MAX, HALF_WORLD_MAX);
    Polys[0].Normal = FVector(0.000000, 0.000000, 1.000000);
    Polys[0].Base = Polys[0].Vertices[0];

    new(Polys[1].Vertices)FVector(-HALF_WORLD_MAX, HALF_WORLD_MAX, -HALF_WORLD_MAX);
    new(Polys[1].Vertices)FVector(HALF_WORLD_MAX, HALF_WORLD_MAX, -HALF_WORLD_MAX);
    new(Polys[1].Vertices)FVector(HALF_WORLD_MAX, -HALF_WORLD_MAX, -HALF_WORLD_MAX);
    new(Polys[1].Vertices)FVector(-HALF_WORLD_MAX, -HALF_WORLD_MAX, -HALF_WORLD_MAX);
    Polys[1].Normal = FVector(0.000000, 0.000000, -1.000000);
    Polys[1].Base = Polys[1].Vertices[0];

    new(Polys[2].Vertices)FVector(-HALF_WORLD_MAX, HALF_WORLD_MAX, -HALF_WORLD_MAX);
    new(Polys[2].Vertices)FVector(-HALF_WORLD_MAX, HALF_WORLD_MAX, HALF_WORLD_MAX);
    new(Polys[2].Vertices)FVector(HALF_WORLD_MAX, HALF_WORLD_MAX, HALF_WORLD_MAX);
    new(Polys[2].Vertices)FVector(HALF_WORLD_MAX, HALF_WORLD_MAX, -HALF_WORLD_MAX);
    Polys[2].Normal = FVector(0.000000, 1.000000, 0.000000);
    Polys[2].Base = Polys[2].Vertices[0];

    new(Polys[3].Vertices)FVector(HALF_WORLD_MAX, -HALF_WORLD_MAX, -HALF_WORLD_MAX);
    new(Polys[3].Vertices)FVector(HALF_WORLD_MAX, -HALF_WORLD_MAX, HALF_WORLD_MAX);
    new(Polys[3].Vertices)FVector(-HALF_WORLD_MAX, -HALF_WORLD_MAX, HALF_WORLD_MAX);
    new(Polys[3].Vertices)FVector(-HALF_WORLD_MAX, -HALF_WORLD_MAX, -HALF_WORLD_MAX);
    Polys[3].Normal = FVector(0.000000, -1.000000, 0.000000);
    Polys[3].Base = Polys[3].Vertices[0];

    new(Polys[4].Vertices)FVector(HALF_WORLD_MAX, HALF_WORLD_MAX, -HALF_WORLD_MAX);
    new(Polys[4].Vertices)FVector(HALF_WORLD_MAX, HALF_WORLD_MAX, HALF_WORLD_MAX);
    new(Polys[4].Vertices)FVector(HALF_WORLD_MAX, -HALF_WORLD_MAX, HALF_WORLD_MAX);
    new(Polys[4].Vertices)FVector(HALF_WORLD_MAX, -HALF_WORLD_MAX, -HALF_WORLD_MAX);
    Polys[4].Normal = FVector(1.000000, 0.000000, 0.000000);
    Polys[4].Base = Polys[4].Vertices[0];

    new(Polys[5].Vertices)FVector(-HALF_WORLD_MAX, -HALF_WORLD_MAX, -HALF_WORLD_MAX);
    new(Polys[5].Vertices)FVector(-HALF_WORLD_MAX, -HALF_WORLD_MAX, HALF_WORLD_MAX);
    new(Polys[5].Vertices)FVector(-HALF_WORLD_MAX, HALF_WORLD_MAX, HALF_WORLD_MAX);
    new(Polys[5].Vertices)FVector(-HALF_WORLD_MAX, HALF_WORLD_MAX, -HALF_WORLD_MAX);
    Polys[5].Normal = FVector(-1.000000, 0.000000, 0.000000);
    Polys[5].Base = Polys[5].Vertices[0];
    // Empty hulls.
    //Model->LeafHulls.Empty();
    for (int32 i = 0; i < Model->Nodes.Num(); i++)
        Model->Nodes[i].iCollisionBound = INDEX_NONE;
    FilterBound(Model, NULL, 0, PolyList, 6, Model->RootOutside);
}

void StaticMeshGenerator::TagReferencedNodes(UModel *Model, int32 *NodeRef, int32 *PolyRef, int32 iNode)
{
    FBspNode &Node = Model->Nodes[iNode];

    NodeRef[iNode] = 0;
    PolyRef[Node.iSurf] = 0;

    if (Node.iFront != INDEX_NONE) TagReferencedNodes(Model, NodeRef, PolyRef, Node.iFront);
    if (Node.iBack != INDEX_NONE) TagReferencedNodes(Model, NodeRef, PolyRef, Node.iBack);
    if (Node.iPlane != INDEX_NONE) TagReferencedNodes(Model, NodeRef, PolyRef, Node.iPlane);
}

void StaticMeshGenerator::SplitPartitioner(UModel* Model, FPoly** PolyList, FPoly** FrontList, FPoly** BackList, int32 n, int32 nPolys, int32& nFront, int32& nBack, FPoly InfiniteEdPoly, TArray<FPoly*>& AllocatedFPolys)
{
    FPoly FrontPoly, BackPoly;
    while (n < nPolys)
    {
        FPoly* Poly = PolyList[n];
        switch (InfiniteEdPoly.SplitWithPlane(Poly->Vertices[0], Poly->Normal, &FrontPoly, &BackPoly, 0))
        {
        case SP_Coplanar:
            // May occasionally happen.
            //				UE_LOG(LogBSPOps, Log,  TEXT("FilterBound: Got inficoplanar") );
            break;

        case SP_Front:
            // Shouldn't happen if hull is correct.
            //				UE_LOG(LogBSPOps, Log,  TEXT("FilterBound: Got infifront") );
            return;

        case SP_Split:
            InfiniteEdPoly = BackPoly;
            break;

        case SP_Back:
            break;
        }
        n++;
    }

    FPoly* New = new FPoly;
    *New = InfiniteEdPoly;
    New->Reverse();
    New->iBrushPoly |= 0x40000000;
    FrontList[nFront++] = New;
    AllocatedFPolys.Add(New);

    New = new FPoly;
    *New = InfiniteEdPoly;
    BackList[nBack++] = New;
    AllocatedFPolys.Add(New);
}

void StaticMeshGenerator::FilterBound(UModel* Model, FBox* ParentBound, int32 iNode, FPoly** PolyList, int32 nPolys, int32 Outside)
{
    FMemMark Mark(FMemStack::Get());
    FBspNode&	Node = Model->Nodes[iNode];
    FBspSurf&	Surf = Model->Surfs[Node.iSurf];
    FVector		Base = Surf.Plane * Surf.Plane.W;
    FVector&	Normal = Model->Vectors[Surf.vNormal];
    FBox		Bound(ForceInit);

    Bound.Min.X = Bound.Min.Y = Bound.Min.Z = +WORLD_MAX;
    Bound.Max.X = Bound.Max.Y = Bound.Max.Z = -WORLD_MAX;

    // Split bound into front half and back half.
    FPoly** FrontList = new(FMemStack::Get(), nPolys * 2 + 16)FPoly*; int32 nFront = 0;
    FPoly** BackList = new(FMemStack::Get(), nPolys * 2 + 16)FPoly*; int32 nBack = 0;

    // Keeping track of allocated FPoly structures to delete later on.
    TArray<FPoly*> AllocatedFPolys;

    FPoly* FrontPoly = new FPoly;
    FPoly* BackPoly = new FPoly;

    // Keep track of allocations.
    AllocatedFPolys.Add(FrontPoly);
    AllocatedFPolys.Add(BackPoly);

    for (int32 i = 0; i < nPolys; i++)
    {
        FPoly *Poly = PolyList[i];
        switch (Poly->SplitWithPlane(Base, Normal, FrontPoly, BackPoly, 0))
        {
        case SP_Coplanar:
            //				UE_LOG(LogBSPOps, Log,  TEXT("FilterBound: Got coplanar") );
            FrontList[nFront++] = Poly;
            BackList[nBack++] = Poly;
            break;

        case SP_Front:
            FrontList[nFront++] = Poly;
            break;

        case SP_Back:
            BackList[nBack++] = Poly;
            break;

        case SP_Split:
            FrontList[nFront++] = FrontPoly;
            BackList[nBack++] = BackPoly;

            FrontPoly = new FPoly;
            BackPoly = new FPoly;

            // Keep track of allocations.
            AllocatedFPolys.Add(FrontPoly);
            AllocatedFPolys.Add(BackPoly);

            break;

        default:
            //UE_LOG(LogBSPOps, Fatal, TEXT("FZoneFilter::FilterToLeaf: Unknown split code"));
            break;
        }
    }
    if (nFront && nBack)
    {
        // Add partitioner plane to front and back.
        FPoly InfiniteEdPoly = BuildInfiniteFPoly(Model, iNode);
        InfiniteEdPoly.iBrushPoly = iNode;

        SplitPartitioner(Model, PolyList, FrontList, BackList, 0, nPolys, nFront, nBack, InfiniteEdPoly, AllocatedFPolys);
    }
    else
    {
        // 		if( !nFront ) UE_LOG(LogBSPOps, Log,  TEXT("FilterBound: Empty fronthull") );
        // 		if( !nBack  ) UE_LOG(LogBSPOps, Log,  TEXT("FilterBound: Empty backhull") );
    }

    // Recursively update all our childrens' bounding volumes.
    if (nFront > 0)
    {
        if (Node.iFront != INDEX_NONE)
            FilterBound(Model, &Bound, Node.iFront, FrontList, nFront, Outside || Node.IsCsg());
        else if (Outside || Node.IsCsg())
            UpdateBoundWithPolys(Bound, FrontList, nFront);
        else
            UpdateConvolutionWithPolys(Model, iNode, FrontList, nFront);
    }
    if (nBack > 0)
    {
        if (Node.iBack != INDEX_NONE)
            FilterBound(Model, &Bound, Node.iBack, BackList, nBack, Outside && !Node.IsCsg());
        else if (Outside && !Node.IsCsg())
            UpdateBoundWithPolys(Bound, BackList, nBack);
        else
            UpdateConvolutionWithPolys(Model, iNode, BackList, nBack);
    }

    // Update parent bound to enclose this bound.
    if (ParentBound)
        *ParentBound += Bound;

    // Delete FPolys allocated above. We cannot use FMemStack::Get() for FPoly as the array data FPoly contains will be allocated in regular memory.
    for (int32 i = 0; i < AllocatedFPolys.Num(); i++)
    {
        FPoly* AllocatedFPoly = AllocatedFPolys[i];
        delete AllocatedFPoly;
    }

    Mark.Pop();
}

FPoly StaticMeshGenerator::BuildInfiniteFPoly(UModel* Model, int32 iNode)
{
    FBspNode &Node = Model->Nodes[iNode];
    FBspSurf &Poly = Model->Surfs[Node.iSurf];
    FVector  Base = Poly.Plane * Poly.Plane.W;
    FVector  Normal = Poly.Plane;
    FVector	 Axis1, Axis2;

    // Find two non-problematic axis vectors.
    Normal.FindBestAxisVectors(Axis1, Axis2);

    // Set up the FPoly.
    FPoly EdPoly;
    EdPoly.Init();
    EdPoly.Normal = Normal;
    EdPoly.Base = Base;
    new(EdPoly.Vertices) FVector(Base + Axis1*WORLD_MAX + Axis2*WORLD_MAX);
    new(EdPoly.Vertices) FVector(Base - Axis1*WORLD_MAX + Axis2*WORLD_MAX);
    new(EdPoly.Vertices) FVector(Base - Axis1*WORLD_MAX - Axis2*WORLD_MAX);
    new(EdPoly.Vertices) FVector(Base + Axis1*WORLD_MAX - Axis2*WORLD_MAX);

    return EdPoly;
}

void StaticMeshGenerator::UpdateBoundWithPolys(FBox& Bound, FPoly** PolyList, int32 nPolys)
{
    for (int32 i = 0; i < nPolys; i++)
        for (int32 j = 0; j < PolyList[i]->Vertices.Num(); j++)
            Bound += PolyList[i]->Vertices[j];
}

void StaticMeshGenerator::UpdateConvolutionWithPolys(UModel *Model, int32 iNode, FPoly **PolyList, int32 nPolys)
{
    FBox Box(ForceInit);

    FBspNode &Node = Model->Nodes[iNode];
    //Node.iCollisionBound = Model->LeafHulls.Num();
    for (int32 i = 0; i < nPolys; i++)
    {
        if (PolyList[i]->iBrushPoly != INDEX_NONE)
        {
            int32 j;
            for (j = 0; j < i; j++)
                if (PolyList[j]->iBrushPoly == PolyList[i]->iBrushPoly)
                    break;
            //if (j >= i)
            //    Model->LeafHulls.Add(PolyList[i]->iBrushPoly);
            //    ;
        }
        for (int32 j = 0; j < PolyList[i]->Vertices.Num(); j++)
            Box += PolyList[i]->Vertices[j];
    }
    //Model->LeafHulls.Add(INDEX_NONE);

    //// Add bounds.
    //Model->LeafHulls.Add(*(int32*)&Box.Min.X);
    //Model->LeafHulls.Add(*(int32*)&Box.Min.Y);
    //Model->LeafHulls.Add(*(int32*)&Box.Min.Z);
    //Model->LeafHulls.Add(*(int32*)&Box.Max.X);
    //Model->LeafHulls.Add(*(int32*)&Box.Max.Y);
    //Model->LeafHulls.Add(*(int32*)&Box.Max.Z);

}

void StaticMeshGenerator::SplitPolyList(UModel *Model, int32 iParent, ENodePlace NodePlace, int32 NumPolys, FPoly **PolyList, EBspOptimization Opt, int32 Balance, int32 PortalBias, int32 RebuildSimplePolys, FBspPointsGridContainer* GridContainer)
{
    FMemMark Mark(FMemStack::Get());

    // Keeping track of allocated FPoly structures to delete later on.
    TArray<FPoly*> AllocatedFPolys;

    // To account for big EdPolys split up.
    int32 NumPolysToAlloc = NumPolys + 8 + NumPolys / 4;
    int32 NumFront = 0; FPoly **FrontList = new(FMemStack::Get(), NumPolysToAlloc)FPoly*;
    int32 NumBack = 0; FPoly **BackList = new(FMemStack::Get(), NumPolysToAlloc)FPoly*;

    FPoly *SplitPoly = FindBestSplit(NumPolys, PolyList, Opt, Balance, PortalBias);

    // Add the splitter poly to the Bsp with either a new BspSurf or an existing one.
    if (RebuildSimplePolys)
    {
        SplitPoly->iLinkSurf = Model->Surfs.Num();
    }

    int32 iOurNode = BspAddNode(Model, iParent, NodePlace, 0, SplitPoly, GridContainer);
    int32 iPlaneNode = iOurNode;

    // Now divide all polygons in the pool into (A^2 polygons that are
    // in front of Poly, and (B) polygons that are in back of Poly.
    // Coplanar polys are inserted immediately, before recursing.

    // If any polygons are split by Poly, we ignrore the original poly,
    // split it into two polys, and add two new polys to the pool.
    FPoly *FrontEdPoly = new FPoly;
    FPoly *BackEdPoly = new FPoly;
    // Keep track of allocations.
    AllocatedFPolys.Add(FrontEdPoly);
    AllocatedFPolys.Add(BackEdPoly);

    for (int32 i = 0; i < NumPolys; i++)
    {
        FPoly *EdPoly = PolyList[i];
        if (EdPoly == SplitPoly)
        {
            continue;
        }

        int x = SplitPoly->Vertices[0].X;
        int y = SplitPoly->Vertices[0].Y;
        int z = SplitPoly->Vertices[0].Z;

        switch (EdPoly->SplitWithPlane(SplitPoly->Vertices[0], SplitPoly->Normal, FrontEdPoly, BackEdPoly, 1))
        {
        case SP_Coplanar:
            if (RebuildSimplePolys)
            {
                EdPoly->iLinkSurf = Model->Surfs.Num() - 1;
            }
            iPlaneNode = BspAddNode(Model, iPlaneNode, NODE_Plane, 0, EdPoly, GridContainer);
            break;

        case SP_Front:
            FrontList[NumFront++] = PolyList[i];
            break;

        case SP_Back:
            BackList[NumBack++] = PolyList[i];
            break;

        case SP_Split:

            // Create front & back nodes.
            FrontList[NumFront++] = FrontEdPoly;
            BackList[NumBack++] = BackEdPoly;

            FrontEdPoly = new FPoly;
            BackEdPoly = new FPoly;
            // Keep track of allocations.
            AllocatedFPolys.Add(FrontEdPoly);
            AllocatedFPolys.Add(BackEdPoly);

            break;
        }
    }

    // Recursively split the front and back pools.
    if (NumFront > 0) SplitPolyList(Model, iOurNode, NODE_Front, NumFront, FrontList, Opt, Balance, PortalBias, RebuildSimplePolys, GridContainer);
    if (NumBack > 0) SplitPolyList(Model, iOurNode, NODE_Back, NumBack, BackList, Opt, Balance, PortalBias, RebuildSimplePolys, GridContainer);

    // Delete FPolys allocated above. We cannot use FMemStack::Get() for FPoly as the array data FPoly contains will be allocated in regular memory.
    for (int32 i = 0; i < AllocatedFPolys.Num(); i++)
    {
        FPoly* AllocatedFPoly = AllocatedFPolys[i];
        delete AllocatedFPoly;
    }

    Mark.Pop();
}

FPoly* StaticMeshGenerator::FindBestSplit(int32 NumPolys, FPoly** PolyList, EBspOptimization Opt, int32 Balance, int32 InPortalBias)
{
    check(NumPolys > 0);

    // No need to test if only one poly.
    if (NumPolys == 1)
        return PolyList[0];

    FPoly   *Poly, *Best = NULL;
    float   Score, BestScore;
    int32     i, Index, j, Inc;
    int32     Splits, Front, Back, Coplanar, AllSemiSolids;

    //PortalBias -- added by Legend on 4/12/2000
    float	PortalBias = InPortalBias / 100.0f;
    Balance &= 0xFF;								// keep only the low byte to recover "Balance"
                                                    //UE_LOG(LogBSPOps, Log, TEXT("Balance=%d PortalBias=%f"), Balance, PortalBias );

    if (Opt == BSP_Optimal)  Inc = 1;					// Test lots of nodes.
    else if (Opt == BSP_Good)		Inc = FMath::Max(1, NumPolys / 20);	// Test 20 nodes.
    else /* BSP_Lame */			Inc = FMath::Max(1, NumPolys / 4);	// Test 4 nodes.

                                                                    // See if there are any non-semisolid polygons here.
    for (i = 0; i < NumPolys; i++)
        if (!(PolyList[i]->PolyFlags & PF_AddLast))
            break;
    AllSemiSolids = (i >= NumPolys);

    // Search through all polygons in the pool and find:
    // A. The number of splits each poly would make.
    // B. The number of front and back nodes the polygon would create.
    // C. Number of coplanars.
    BestScore = 0;
    for (i = 0; i < NumPolys; i += Inc)
    {
        Splits = Front = Back = Coplanar = 0;
        Index = i - 1;
        do
        {
            Index++;
            Poly = PolyList[Index];
        } while (Index < (i + Inc) && Index < NumPolys
            && ((Poly->PolyFlags & PF_AddLast) && !(Poly->PolyFlags & PF_Portal))
            && !AllSemiSolids);
        if (Index >= i + Inc || Index >= NumPolys)
            continue;

        for (j = 0; j < NumPolys; j += Inc) if (j != Index)
        {
            FPoly *OtherPoly = PolyList[j];
            switch (OtherPoly->SplitWithPlaneFast(FPlane(Poly->Vertices[0], Poly->Normal), NULL, NULL))
            {
            case SP_Coplanar:
                Coplanar++;
                break;

            case SP_Front:
                Front++;
                break;

            case SP_Back:
                Back++;
                break;

            case SP_Split:
                // Disfavor splitting polys that are zone portals.
                if (!(OtherPoly->PolyFlags & PF_Portal))
                    Splits++;
                else
                    Splits += 16;
                break;
            }
        }
        // added by Legend 1/31/1999
        // Score optimization: minimize cuts vs. balance tree (as specified in BSP Rebuilder dialog)
        Score = (100.0 - float(Balance)) * Splits + float(Balance) * FMath::Abs(Front - Back);
        if (Poly->PolyFlags & PF_Portal)
        {
            // PortalBias -- added by Legend on 4/12/2000
            //
            // PortalBias enables level designers to control the effect of Portals on the BSP.
            // This effect can range from 0.0 (ignore portals), to 1.0 (portals cut everything).
            //
            // In builds prior to this (since the 221 build dating back to 1/31/1999) the bias
            // has been 1.0 causing the portals to cut the BSP in ways that will potentially
            // degrade level performance, and increase the BSP complexity.
            // 
            // By setting the bias to a value between 0.3 and 0.7 the positive effects of 
            // the portals are preserved without giving them unreasonable priority in the BSP.
            //
            // Portals should be weighted high enough in the BSP to separate major parts of the
            // level from each other (pushing entire rooms down the branches of the BSP), but
            // should not be so high that portals cut through adjacent geometry in a way that
            // increases complexity of the room being (typically, accidentally) cut.
            //
            Score -= (100.0 - float(Balance)) * Splits * PortalBias; // ignore PortalBias of the split polys -- bias toward portal selection for cutting planes!
        }
        //UE_LOG(LogBSPOps, Log,  "  %4d: Score = %f (Front = %4d, Back = %4d, Splits = %4d, Flags = %08X)", Index, Score, Front, Back, Splits, Poly->PolyFlags ); //LEC

        if (Score < BestScore || !Best)
        {
            Best = Poly;
            BestScore = Score;
        }
    }
    check(Best);
    return Best;
}

int32 StaticMeshGenerator::BspAddNode(UModel* Model, int32 iParent, ENodePlace NodePlace, uint32 NodeFlags, FPoly* EdPoly, FBspPointsGridContainer* GridContainer)
{
    if (NodePlace == NODE_Plane)
    {
        // Make sure coplanars are added at the end of the coplanar list so that 
        // we don't insert NF_IsNew nodes with non NF_IsNew coplanar children.
        while (Model->Nodes[iParent].iPlane != INDEX_NONE)
        {
            iParent = Model->Nodes[iParent].iPlane;
        }
    }
    FBspSurf* Surf = NULL;
    if (EdPoly->iLinkSurf == Model->Surfs.Num())
    {
        int32 NewIndex = Model->Surfs.AddZeroed();
        Surf = &Model->Surfs[NewIndex];

        // This node has a new polygon being added by bspBrushCSG; must set its properties here.
        Surf->pBase = BspAddPoint(Model, &EdPoly->Base, GridContainer, 1);
        Surf->vNormal = BspAddVector(Model, &EdPoly->Normal, GridContainer, 1);
        Surf->vTextureU = BspAddVector(Model, &EdPoly->TextureU, GridContainer, 0);
        Surf->vTextureV = BspAddVector(Model, &EdPoly->TextureV, GridContainer, 0);
        Surf->Material = EdPoly->Material;
        Surf->Actor = NULL;

        Surf->PolyFlags = EdPoly->PolyFlags & ~PF_NoAddToBSP;
        Surf->LightMapScale = EdPoly->LightMapScale;

        // Find the LightmassPrimitiveSettings in the UModel...
        int32 FoundLightmassIndex = INDEX_NONE;
        if (Model->LightmassSettings.Find(EdPoly->LightmassSettings, FoundLightmassIndex) == false)
        {
            FoundLightmassIndex = Model->LightmassSettings.Add(EdPoly->LightmassSettings);
        }
        Surf->iLightmassIndex = FoundLightmassIndex;

        Surf->Actor = EdPoly->Actor;
        Surf->iBrushPoly = EdPoly->iBrushPoly;

        Surf->Plane = FPlane(EdPoly->Vertices[0], EdPoly->Normal);
    }
    else
    {
        check(EdPoly->iLinkSurf != INDEX_NONE);
        check(EdPoly->iLinkSurf < Model->Surfs.Num());
        Surf = &Model->Surfs[EdPoly->iLinkSurf];
    }

    // Set NodeFlags.
    if (Surf->PolyFlags & PF_NotSolid) NodeFlags |= NF_NotCsg;
    if (Surf->PolyFlags & (PF_Invisible | PF_Portal)) NodeFlags |= NF_NotVisBlocking;

    if (EdPoly->Vertices.Num() > FBspNode::MAX_NODE_VERTICES)
    {
        // Split up into two coplanar sub-polygons (one with MAX_NODE_VERTICES vertices and
        // one with all the remaining vertices) and recursively add them.

        // EdPoly1 is just the first MAX_NODE_VERTICES from EdPoly.
        FMemMark Mark(FMemStack::Get());
        FPoly *EdPoly1 = new FPoly;
        *EdPoly1 = *EdPoly;
        EdPoly1->Vertices.RemoveAt(FBspNode::MAX_NODE_VERTICES, EdPoly->Vertices.Num() - FBspNode::MAX_NODE_VERTICES);

        // EdPoly2 is the first vertex from EdPoly, and the last EdPoly->Vertices.Num() - MAX_NODE_VERTICES + 1.
        FPoly *EdPoly2 = new FPoly;
        *EdPoly2 = *EdPoly;
        EdPoly2->Vertices.RemoveAt(1, FBspNode::MAX_NODE_VERTICES - 2);

        int32 iNode = BspAddNode(Model, iParent, NodePlace, NodeFlags, EdPoly1, GridContainer); // Add this poly first.
        BspAddNode(Model, iNode, NODE_Plane, NodeFlags, EdPoly2, GridContainer); // Then add other (may be bigger).

        delete EdPoly1;
        delete EdPoly2;

        Mark.Pop();
        return iNode; // Return coplanar "parent" node (not coplanar child)
    }
    else
    {
        // Add node.
        int32 iNode = Model->Nodes.AddZeroed();
        FBspNode& Node = Model->Nodes[iNode];

        // Tell transaction tracking system that parent is about to be modified.
        FBspNode* Parent = NULL;
        if (NodePlace != NODE_Root)
            Parent = &Model->Nodes[iParent];

        // Set node properties.
        Node.iSurf = EdPoly->iLinkSurf;
        Node.NodeFlags = NodeFlags;
        Node.iCollisionBound = INDEX_NONE;
        Node.Plane = FPlane(EdPoly->Vertices[0], EdPoly->Normal);
        Node.iVertPool = Model->Verts.AddUninitialized(EdPoly->Vertices.Num());
        Node.iFront = INDEX_NONE;
        Node.iBack = INDEX_NONE;
        Node.iPlane = INDEX_NONE;
        if (NodePlace == NODE_Root)
        {
            Node.iLeaf[0] = INDEX_NONE;
            Node.iLeaf[1] = INDEX_NONE;
            Node.iZone[0] = 0;
            Node.iZone[1] = 0;
        }
        else if (NodePlace == NODE_Front || NodePlace == NODE_Back)
        {
            int32 ZoneFront = NodePlace == NODE_Front;
            Node.iLeaf[0] = Parent->iLeaf[ZoneFront];
            Node.iLeaf[1] = Parent->iLeaf[ZoneFront];
            Node.iZone[0] = Parent->iZone[ZoneFront];
            Node.iZone[1] = Parent->iZone[ZoneFront];
        }
        else
        {
            int32 IsFlipped = (Node.Plane | Parent->Plane) < 0.0;
            Node.iLeaf[0] = Parent->iLeaf[IsFlipped];
            Node.iLeaf[1] = Parent->iLeaf[1 - IsFlipped];
            Node.iZone[0] = Parent->iZone[IsFlipped];
            Node.iZone[1] = Parent->iZone[1 - IsFlipped];
        }

        // Link parent to this node.
        if (NodePlace == NODE_Front)
        {
            Parent->iFront = iNode;
        }
        else if (NodePlace == NODE_Back)
        {
            Parent->iBack = iNode;
        }
        else if (NodePlace == NODE_Plane)
        {
            Parent->iPlane = iNode;
        }

        // Add all points to point table, merging nearly-overlapping polygon points
        // with other points in the poly to prevent criscrossing vertices from
        // being generated.

        // Must maintain Node->NumVertices on the fly so that bspAddPoint is always
        // called with the Bsp in a clean state.
        Node.NumVertices = 0;
        FVert* VertPool = &Model->Verts[Node.iVertPool];
        for (uint8 i = 0; i < EdPoly->Vertices.Num(); i++)
        {
            int32 pVertex = BspAddPoint(Model, &EdPoly->Vertices[i], GridContainer, 0);
            if (Node.NumVertices == 0 || VertPool[Node.NumVertices - 1].pVertex != pVertex)
            {
                VertPool[Node.NumVertices].iSide = INDEX_NONE;
                VertPool[Node.NumVertices].pVertex = pVertex;
                Node.NumVertices++;
            }
        }
        if (Node.NumVertices >= 2 && VertPool[0].pVertex == VertPool[Node.NumVertices - 1].pVertex)
        {
            Node.NumVertices--;
        }
        if (Node.NumVertices < 3)
        {
            //GErrors++;
            Node.NumVertices = 0;
        }

        return iNode;
    }
}

int32 StaticMeshGenerator::BspAddPoint(UModel* Model, FVector* V, FBspPointsGridContainer* GridContainer, bool Exact)
{
    const float Thresh = Exact ? THRESH_POINTS_ARE_SAME : THRESH_POINTS_ARE_NEAR;

    if (GridContainer->PointsGrid)
    {
        // If a points grid has been built for quick point lookup, use that instead of doing a linear search
        const int32 NextIndex = Model->Points.Num();
        // Always look for points with a low threshold; a generous threshold can result in 'leaks' in the BSP and unwanted polys being generated
        const int32 ReturnedIndex = GridContainer->PointsGrid->FindOrAddPoint(*V, NextIndex, THRESH_POINTS_ARE_SAME);
        if (ReturnedIndex == NextIndex)
        {
            Model->Points.Add(*V);
        }

        return ReturnedIndex;
    }

    // Try to find a match quickly from the Bsp. This finds all potential matches
    // except for any dissociated from nodes/surfaces during a rebuild.
    FVector Temp;
    int32 pVertex;
    float NearestDist = Model->FindNearestVertex(*V, Temp, Thresh, pVertex);
    if ((NearestDist >= 0.0) && (NearestDist <= Thresh))
    {
        // Found an existing point.
        return pVertex;
    }
    else
    {
        // No match found; add it slowly to find duplicates.
        return AddThing(Model->Points, *V, Thresh, false);
    }
}

int32 StaticMeshGenerator::BspAddVector(UModel* Model, FVector* V, FBspPointsGridContainer* GridContainer, bool Exact)
{
    const float Thresh = Exact ? THRESH_NORMALS_ARE_SAME : THRESH_VECTORS_ARE_NEAR;

    if (GridContainer->VectorsGrid)
    {
        // If a points grid has been built for quick vector lookup, use that instead of doing a linear search
        const int32 NextIndex = Model->Vectors.Num();
        const int32 ReturnedIndex = GridContainer->VectorsGrid->FindOrAddPoint(*V, NextIndex, Thresh);
        if (ReturnedIndex == NextIndex)
        {
            Model->Vectors.Add(*V);
        }

        return ReturnedIndex;
    }

    return AddThing
    (
        Model->Vectors,
        *V,
        Exact ? THRESH_NORMALS_ARE_SAME : THRESH_VECTORS_ARE_NEAR,
        1
    );
}

int32 StaticMeshGenerator::AddThing(TArray<FVector>& Vectors, FVector& V, float Thresh, int32 Check)
{
    if (Check)
    {
        // See if this is very close to an existing point/vector.		
        for (int32 i = 0; i < Vectors.Num(); i++)
        {
            const FVector &TableVect = Vectors[i];
            float Temp = (V.X - TableVect.X);
            if ((Temp > -Thresh) && (Temp < Thresh))
            {
                Temp = (V.Y - TableVect.Y);
                if ((Temp > -Thresh) && (Temp < Thresh))
                {
                    Temp = (V.Z - TableVect.Z);
                    if ((Temp > -Thresh) && (Temp < Thresh))
                    {
                        // Found nearly-matching vector.
                        return i;
                    }
                }
            }
        }
    }
    return Vectors.Add(V);
}

void StaticMeshGenerator::FBspPointsGrid::Clear(int32 InitialSize)
{
    GridMap.Empty(InitialSize);
}

int32 StaticMeshGenerator::FBspPointsGrid::FindOrAddPoint(const FVector& Point, int32 Index, float PointThreshold)
{
    // Offset applied to the grid coordinates so aligned vertices (the normal case) don't overlap several grid items (taking into account the threshold)
    const float GridOffset = 0.12345f;

    const float AdjustedPointX = Point.X - GridOffset;
    const float AdjustedPointY = Point.Y - GridOffset;
    const float AdjustedPointZ = Point.Z - GridOffset;

    const float GridX = AdjustedPointX * OneOverGranularity;
    const float GridY = AdjustedPointY * OneOverGranularity;
    const float GridZ = AdjustedPointZ * OneOverGranularity;

    // Get the grid indices corresponding to the point coordinates
    const int32 GridIndexX = FMath::FloorToInt(GridX);
    const int32 GridIndexY = FMath::FloorToInt(GridY);
    const int32 GridIndexZ = FMath::FloorToInt(GridZ);

    // Find grid item in map
    FBspPointsGridItem& GridItem = GridMap.FindOrAdd(FBspPointsKey(GridIndexX, GridIndexY, GridIndexZ));

    // Iterate through grid item points and return a point if it's close to the threshold
    const float PointThresholdSquared = PointThreshold * PointThreshold;
    for (const FBspIndexedPoint& IndexedPoint : GridItem.IndexedPoints)
    {
        if (FVector::DistSquared(IndexedPoint.Point, Point) <= PointThresholdSquared)
        {
            return IndexedPoint.Index;
        }
    }

    // Otherwise, the point is new: add it to the grid item.
    GridItem.IndexedPoints.Emplace(Point, Index);

    // The grid has a maximum threshold of a certain radius. If the point is near the edge of a grid cube, it may overlap into other items.
    // Add it to all grid items it can be seen from.
    const float GridThreshold = Threshold * OneOverGranularity;
    const int32 NeighbourX = GetAdjacentIndexIfOverlapping(GridIndexX, GridX, GridThreshold);
    const int32 NeighbourY = GetAdjacentIndexIfOverlapping(GridIndexY, GridY, GridThreshold);
    const int32 NeighbourZ = GetAdjacentIndexIfOverlapping(GridIndexZ, GridZ, GridThreshold);

    const bool bOverlapsInX = (NeighbourX != GridIndexX);
    const bool bOverlapsInY = (NeighbourY != GridIndexY);
    const bool bOverlapsInZ = (NeighbourZ != GridIndexZ);

    if (bOverlapsInX)
    {
        GridMap.FindOrAdd(FBspPointsKey(NeighbourX, GridIndexY, GridIndexZ)).IndexedPoints.Emplace(Point, Index);

        if (bOverlapsInY)
        {
            GridMap.FindOrAdd(FBspPointsKey(NeighbourX, NeighbourY, GridIndexZ)).IndexedPoints.Emplace(Point, Index);

            if (bOverlapsInZ)
            {
                GridMap.FindOrAdd(FBspPointsKey(NeighbourX, NeighbourY, NeighbourZ)).IndexedPoints.Emplace(Point, Index);
            }
        }
        else if (bOverlapsInZ)
        {
            GridMap.FindOrAdd(FBspPointsKey(NeighbourX, GridIndexY, NeighbourZ)).IndexedPoints.Emplace(Point, Index);
        }
    }
    else
    {
        if (bOverlapsInY)
        {
            GridMap.FindOrAdd(FBspPointsKey(GridIndexX, NeighbourY, GridIndexZ)).IndexedPoints.Emplace(Point, Index);

            if (bOverlapsInZ)
            {
                GridMap.FindOrAdd(FBspPointsKey(GridIndexX, NeighbourY, NeighbourZ)).IndexedPoints.Emplace(Point, Index);
            }
        }
        else if (bOverlapsInZ)
        {
            GridMap.FindOrAdd(FBspPointsKey(GridIndexX, GridIndexY, NeighbourZ)).IndexedPoints.Emplace(Point, Index);
        }
    }

    return Index;
}

FORCEINLINE int32 StaticMeshGenerator::FBspPointsGrid::GetAdjacentIndexIfOverlapping(int32 GridIndex, float GridPos, float GridThreshold)
{
    if (GridPos - GridIndex < GridThreshold)
    {
        return GridIndex - 1;
    }
    else if (1.0f - (GridPos - GridIndex) < GridThreshold)
    {
        return GridIndex + 1;
    }
    else
    {
        return GridIndex;
    }
}