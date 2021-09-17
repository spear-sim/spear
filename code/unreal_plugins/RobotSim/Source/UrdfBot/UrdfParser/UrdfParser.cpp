#include "UrdfParser.h"

void UrdfParser::Parse(FString FileName)
{
    this->links_.Empty();
    this->joints_.Empty();
    this->forces_.Empty();

    FXmlFile file;
    file.LoadFile(FileName);

    auto rootNode = file.GetRootNode();

    for (auto node : rootNode->GetChildrenNodes())
    {
        FString tag = node->GetTag();

        if (tag.Equals(TEXT("link")))
        {
            UrdfLinkSpecification* linkSpecification =
                this->ParseLinkSpecification(node);

            if (!linkSpecification)
                throw std::runtime_error(
                    "Unable to create link specification.");

            if (this->links_.Contains(linkSpecification->Name))
                throw std::runtime_error(
                    "Multiple links with name '" +
                    std::string(TCHAR_TO_UTF8(*linkSpecification->Name)) +
                    "'.");

            this->links_.Add(linkSpecification->Name, linkSpecification);
        }
        else if (tag.Equals(TEXT("joint")))
        {
            UrdfJointSpecification* jointSpecification =
                this->ParseJointSpecification(node);

            if (!jointSpecification)
                throw std::runtime_error(
                    "Unable to create joint specification.");

            if (this->joints_.Contains(jointSpecification->Name))
                throw std::runtime_error(
                    "Multiple joints with name '" +
                    std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                    "'.");

            this->joints_.Add(jointSpecification->Name, jointSpecification);
        }
        else if (tag.Equals(TEXT("force")))
        {
            UrdfForceSpecification* forceSpecification =
                this->ParseForceSpecification(node);

            if (!forceSpecification)
                throw std::runtime_error(
                    "Unable to create force specification.");

            if (this->forces_.Contains(forceSpecification->Name))
                throw std::runtime_error(
                    "Multiple force specifications with name '" +
                    std::string(TCHAR_TO_UTF8(*(forceSpecification->Name))) +
                    "'.");

            this->forces_.Add(forceSpecification->Name, forceSpecification);
        }
        else if (tag.Equals(TEXT("material")))
        {
            UrdfMaterialSpecification* materialSpecification =
                this->ParseMaterialSpecification(node);

            if (!materialSpecification)
                throw std::runtime_error(
                    "Unable to create material specification.");

            if (this->materials_.Contains(materialSpecification->Name))
                throw std::runtime_error(
                    "Multiple materials with the name " +
                    std::string(TCHAR_TO_UTF8(*(materialSpecification->Name))) +
                    ".");

            this->materials_.Add(materialSpecification->Name,
                                 materialSpecification);
        }
    }

    // Sanity check: for a robot with N links, there must be N-1 joints.
    if (this->joints_.Num() + 1 != this->links_.Num())
    {
        // TEST: do nothing.
        // throw std::runtime_error("The number of joints parsed is " +
        // std::to_string(this->joints_.Num()) + ", while the number of links
        // parsed is " + std::to_string(this->joints_.Num()) + ". The number of
        // joints must be one less than the number of links.");
    }

    // Fill in link references
    for (auto kvp : this->joints_)
    {
        UrdfJointSpecification* currentJoint = kvp.Value;

        UrdfLinkSpecification* parentLink;
        UrdfLinkSpecification* childLink;

        if (!this->links_.Contains(currentJoint->ParentLinkName))
            throw std::runtime_error(
                "Joint '" + std::string(TCHAR_TO_UTF8(*currentJoint->Name)) +
                "' references link '" +
                std::string(TCHAR_TO_UTF8(*currentJoint->ParentLinkName)) +
                "' as the parent link, which does not exist.");
        if (!this->links_.Contains(currentJoint->ChildLinkName))
            throw std::runtime_error(
                "Joint '" + std::string(TCHAR_TO_UTF8(*currentJoint->Name)) +
                "' references link '" +
                std::string(TCHAR_TO_UTF8(*currentJoint->ChildLinkName)) +
                "' as the child link, which does not exist.");
        if (currentJoint->Mimic != nullptr &&
            !this->joints_.Contains(currentJoint->Mimic->JointName))
            throw std::runtime_error(
                "Joint '" + std::string(TCHAR_TO_UTF8(*currentJoint->Name)) +
                "' references joint '" +
                std::string(TCHAR_TO_UTF8(*currentJoint->Mimic->JointName)) +
                "' as the mimic joint, which does not exist.");

        parentLink = this->links_[currentJoint->ParentLinkName];
        childLink = this->links_[currentJoint->ChildLinkName];

        currentJoint->ParentLinkSpecification = parentLink;
        currentJoint->ChildLinkSpecification = childLink;

        if (childLink->ParentLink != nullptr)
        {
            // throw std::runtime_error("Link '" +
            // std::string(TCHAR_TO_UTF8(*childLink->Name)) + "' has multiple
            // parents.");
        }

        childLink->ParentLink = parentLink;
        parentLink->Children.Add(
            TPair<UrdfLinkSpecification*, UrdfJointSpecification*>(childLink,
                                                                   kvp.Value));

        if (currentJoint->Mimic != nullptr)
        {
            currentJoint->Mimic->Joint =
                this->joints_[currentJoint->Mimic->JointName];
        }
    }

    // There must be exactly one node that has no parent.
    // This represents the root node of the bot.
    TArray<UrdfLinkSpecification*> rootNodes;

    for (auto kvp : this->links_)
    {
        UrdfLinkSpecification* linkSpecification = kvp.Value;
        if (linkSpecification->ParentLink == nullptr)
            rootNodes.Add(linkSpecification);
    }

    if (rootNodes.Num() == 0)
        throw std::runtime_error(
            "There is no root node. There must be at least one node in the "
            "tree that does not have a parent.");
    else if (rootNodes.Num() > 1)
    {
        std::string errorMessage =
            "There are multiple nodes that have no parent node. There can only "
            "be one node that does not have a parent. The nodes that do not "
            "have parents are: ";
        for (auto node : rootNodes)
        {
            errorMessage += "'";
            errorMessage += std::string(TCHAR_TO_UTF8(*node->Name));
            errorMessage += "' ";
        }
        errorMessage += ".";
        throw std::runtime_error(errorMessage);
    }

    // Validate that for each force, the link references point to valid link
    // specifications.
    for (auto kvp : this->forces_)
    {
        UrdfForceSpecification* forceSpecification = kvp.Value;

        UrdfAngularForceSpecification* angularForceSpecification = nullptr;
        UrdfLinearForceSpecification* linearForceSpecification = nullptr;

        switch (forceSpecification->GetForceSpecificationType())
        {
        case FORCE_ANGULAR:
            angularForceSpecification =
                static_cast<UrdfAngularForceSpecification*>(forceSpecification);
            if (!this->links_.Contains(angularForceSpecification->LinkName))
                throw std::runtime_error(
                    "Angular force '" +
                    std::string(
                        TCHAR_TO_UTF8(*(angularForceSpecification->Name))) +
                    "' is attached to link '" +
                    std::string(
                        TCHAR_TO_UTF8(*(angularForceSpecification->LinkName))) +
                    "', which does not exist.");
            break;
        case FORCE_LINEAR:
            linearForceSpecification =
                static_cast<UrdfLinearForceSpecification*>(forceSpecification);
            if (!this->links_.Contains(linearForceSpecification->LinkName))
                throw std::runtime_error(
                    "Linear force '" +
                    std::string(
                        TCHAR_TO_UTF8(*(linearForceSpecification->Name))) +
                    "' is attached to link '" +
                    std::string(
                        TCHAR_TO_UTF8(*(linearForceSpecification->LinkName))) +
                    "', which does not exist.");
            break;
        }
    }

    // Validate that for each link, if a material is specified, it exists.
    TArray<FString> missingMaterials;
    for (auto kvp : this->links_)
    {
        UrdfLinkSpecification* link = kvp.Value;
        if (link->VisualSpecification != nullptr)
        {
            FString materialName = link->VisualSpecification->MaterialName;

            if (materialName.Len() > 0 &&
                !this->materials_.Contains(materialName))
            {
                missingMaterials.Add(materialName);
            }
        }
    }

    if (missingMaterials.Num() > 0)
    {
        std::string error = "Materials ";
        for (auto materialName : missingMaterials)
        {
            error += std::string(TCHAR_TO_UTF8(*(materialName)));
            error += " ";
        }

        error += "are referenced in links, but not found.";
        throw std::runtime_error(error);
    }
}

TMap<FString, UrdfLinkSpecification*> UrdfParser::GetLinks()
{
    return this->links_;
}

TMap<FString, UrdfJointSpecification*> UrdfParser::GetJoints()
{
    return this->joints_;
}

TMap<FString, UrdfForceSpecification*> UrdfParser::GetForces()
{
    return this->forces_;
}

TMap<FString, UrdfMaterialSpecification*> UrdfParser::GetMaterials()
{
    return this->materials_;
}

UrdfLinkSpecification* UrdfParser::ParseLinkSpecification(FXmlNode* linkNode)
{
    auto linkSpecification = new UrdfLinkSpecification();

    bool hasNameAttribute = false;
    bool hasInertialNode = false;
    bool hasVisualNode = false;
    bool hasCollisionNode = false;

    for (auto attr : linkNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("name")))
        {
            if (hasNameAttribute)
                throw std::runtime_error(
                    "Multiple names specified for link. First name: '" +
                    std::string(TCHAR_TO_UTF8(*linkSpecification->Name)) +
                    "'.");

            hasNameAttribute = true;
            linkSpecification->Name = attr.GetValue();
        }
    }

    if (!hasNameAttribute)
        throw std::runtime_error("Link is missing name attribute.");

    for (auto node : linkNode->GetChildrenNodes())
    {
        FString tag = node->GetTag();

        if (tag.Equals(TEXT("inertial")))
        {
            if (hasInertialNode)
            {
                throw std::runtime_error(
                    "Multiple inertial nodes specified for link '" +
                    std::string(TCHAR_TO_UTF8(*linkSpecification->Name)) +
                    "'.");
            }

            hasInertialNode = true;
            linkSpecification->InertialSpecification =
                this->ParseLinkInertialSpecification(node);
        }
        else if (tag.Equals(TEXT("visual")))
        {
            if (hasVisualNode)
            {
                throw std::runtime_error(
                    "Multiple visual nodes specified for link '" +
                    std::string(TCHAR_TO_UTF8(*linkSpecification->Name)) +
                    "'. Although this is technically allowed, limitations in "
                    "the Unreal engine prevent us from efficiently "
                    "implementing this. Please re-write as a series of "
                    "multiple links with FIXED joint constraints.");
            }

            hasVisualNode = true;
            linkSpecification->VisualSpecification =
                this->ParseLinkVisualSpecification(node);
        }
        else if (tag.Equals(TEXT("collision")))
        {
            if (hasCollisionNode)
            {
                throw std::runtime_error(
                    "Multiple collision nodes specified for link '" +
                    std::string(TCHAR_TO_UTF8(*linkSpecification->Name)) +
                    "'. Although this is technically allowed, limitations in "
                    "the Unreal engine prevent us from efficiently "
                    "implementing this. Please re-write as a series of "
                    "multiple links with FIXED joint constraints.");
            }

            hasCollisionNode = true;
            linkSpecification->CollisionSpecification =
                this->ParseLinkCollisionSpecification(node);
        }
    }

    // ToDo  empty link node is permitted!!!
    // if (!hasInertialNode)
    //    throw std::runtime_error("No inertial node specified for link '" +
    //    std::string(TCHAR_TO_UTF8(*linkSpecification->Name)) + "'.");
    // if (!hasVisualNode && !hasCollisionNode)
    //    throw std::runtime_error("No visual or collision node specified for
    //    link '" + std::string(TCHAR_TO_UTF8(*linkSpecification->Name)) +
    //    "'.");

    return linkSpecification;
}

UrdfJointSpecification* UrdfParser::ParseJointSpecification(FXmlNode* jointNode)
{
    auto jointSpecification = new UrdfJointSpecification();

    bool hasNameAttribute = false;
    bool hasTypeAttribute = false;
    bool hasOriginNode = false;
    bool hasParentNode = false;
    bool hasChildNode = false;
    bool hasAxisNode = false;
    bool hasCalibrationNode = false;
    bool hasDynamicsNode = false;
    bool hasLimitNode = false;
    bool hasMimicNode = false;
    bool hasSafetyControllerNode = false;

    for (auto attr : jointNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("name")))
        {
            if (hasNameAttribute)
            {
                throw std::runtime_error(
                    "Multiple name attributes specified for joint. First name: "
                    "'" +
                    std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                    "'.");
            }

            hasNameAttribute = true;
            jointSpecification->Name = attr.GetValue();
        }
        else if (tag.Equals(TEXT("type")))
        {
            if (hasTypeAttribute)
            {
                throw std::runtime_error(
                    "Multiple type attributes specified for joint.");
            }

            hasTypeAttribute = true;
            jointSpecification->Type =
                UrdfJointSpecification::ParseJointType(attr.GetValue());
        }
    }

    if (!hasNameAttribute)
        throw std::runtime_error("No name specified for joint.");
    if (!hasTypeAttribute)
        throw std::runtime_error(
            "No type specified for joint with name '" +
            std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) + "'.");

    for (auto node : jointNode->GetChildrenNodes())
    {
        FString tag = node->GetTag();

        if (tag.Equals(TEXT("origin")))
        {
            if (hasOriginNode)
                throw std::runtime_error(
                    "Multiple origin nodes specified for joint '" +
                    std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                    "'.");

            hasOriginNode = true;
            jointSpecification->Origin = this->ParseUrdfOrigin(node);
        }
        else if (tag.Equals(TEXT("parent")))
        {
            if (hasParentNode)
                throw std::runtime_error(
                    "Multiple parent nodes specified for joint '" +
                    std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                    "'.");

            hasParentNode = true;
            jointSpecification->ParentLinkName =
                node->GetAttribute(TEXT("link"));
        }
        else if (tag.Equals(TEXT("child")))
        {
            if (hasChildNode)
                throw std::runtime_error(
                    "Multiple child nodes specified for joint '" +
                    std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                    "'.");

            hasChildNode = true;
            jointSpecification->ChildLinkName =
                node->GetAttribute(TEXT("link"));
        }
        else if (tag.Equals(TEXT("axis")))
        {
            if (hasAxisNode)
                throw std::runtime_error(
                    "Multiple axis nodes specified for joint '" +
                    std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                    "'.");

            hasAxisNode = true;
            jointSpecification->Axis = this->ParseFVectorFromAttributeString(
                node->GetAttribute(TEXT("xyz")));
            jointSpecification->RollPitchYaw =
                this->ParseFRotatorFromAttributeString(
                    node->GetAttribute(TEXT("rpy")));
        }
        else if (tag.Equals(TEXT("calibration")))
        {
            if (hasCalibrationNode)
                throw std::runtime_error(
                    "Multiple calibration nodes specified for joint '" +
                    std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                    "'.");

            hasCalibrationNode = true;
            jointSpecification->Calibration =
                this->ParseJointCalibrationSpecification(node);
        }
        else if (tag.Equals(TEXT("dynamics")))
        {
            if (hasDynamicsNode)
                throw std::runtime_error(
                    "Multiple dynamics nodes detected for joint '" +
                    std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                    "'.");

            hasDynamicsNode = true;
            jointSpecification->Dynamics =
                this->ParseJointDynamicsSpecification(node);
        }
        else if (tag.Equals(TEXT("limit")))
        {
            if (hasLimitNode)
                throw std::runtime_error(
                    "Multiple limit nodes detected for joint '" +
                    std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                    "'.");

            hasLimitNode = true;
            jointSpecification->Limit =
                this->ParseJointLimitSpecification(node);
        }
        else if (tag.Equals(TEXT("mimic")))
        {
            if (hasMimicNode)
                throw std::runtime_error(
                    "Multiple mimic nodes detected for joint '" +
                    std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                    "'.");

            hasMimicNode = true;
            jointSpecification->Mimic =
                this->ParseJointMimicSpecification(node);
        }
        else if (tag.Equals(TEXT("safety_controller")))
        {
            if (hasSafetyControllerNode)
                throw std::runtime_error(
                    "Multiple safety_controller nodes detected for joint '" +
                    std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
                    "'.");

            hasSafetyControllerNode = true;
            jointSpecification->SafetyController =
                this->ParseJointSafetyControllerSpecification(node);
        }
    }

    if (!hasParentNode)
        throw std::runtime_error(
            "Joint '" + std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
            "' is missing the parent link node.");
    if (!hasChildNode)
        throw std::runtime_error(
            "Joint '" + std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
            "' is missing the child link node.");
    if ((jointSpecification->Type == REVOLUTE_TYPE ||
         jointSpecification->Type == PRISMATIC_TYPE) &&
        !hasLimitNode)
        throw std::runtime_error(
            "Joint '" + std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) +
            "' is of type 'revolute' or 'prismatic', but has no limit "
            "specification. For joints of this type, the limit node must be "
            "provided.");

    return jointSpecification;
}

UrdfForceSpecification* UrdfParser::ParseForceSpecification(FXmlNode* forceNode)
{
    UrdfForceSpecification* forceSpecification = nullptr;

    bool foundName = false;
    FString name;

    for (auto attr : forceNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("name")))
        {
            foundName = true;
            name = attr.GetValue();
        }
        else if (tag.Equals(TEXT("type")))
        {
            if (forceSpecification != nullptr)
            {
                throw std::runtime_error("Multiple force types specified.");
            }

            if (tag.Equals(TEXT("actuator")))
            {
                forceSpecification =
                    this->ParseAngularForceSpecification(forceNode);
            }
            else if (tag.Equals(TEXT("linear")))
            {
                forceSpecification =
                    this->ParseLinearForceSpecification(forceNode);
            }
            else if (tag.Equals(TEXT("angular")))
            {
                forceSpecification =
                    this->ParseAngularForceSpecification(forceNode);
            }
            else
            {
                throw std::runtime_error("Unrecognized force type " +
                                         std::string(TCHAR_TO_UTF8(*tag)));
            }
        }
    }

    if (forceSpecification == nullptr)
    {
        throw std::runtime_error("No force type specified.");
    }

    if (!foundName)
    {
        throw std::runtime_error("Force specified without name.");
    }

    forceSpecification->Name = name;
    return forceSpecification;
}

UrdfMaterialSpecification*
UrdfParser::ParseMaterialSpecification(FXmlNode* materialNode)
{
    UrdfMaterialSpecification* materialSpecification =
        new UrdfMaterialSpecification();

    bool hasName = false;
    for (auto attr : materialNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("name")))
        {
            if (hasName)
            {
                throw std::runtime_error(
                    "Multiple names for material " +
                    std::string(TCHAR_TO_UTF8(*materialSpecification->Name)));
            }

            hasName = true;
            materialSpecification->Name = attr.GetValue();
        }
    }

    for (auto node : materialNode->GetChildrenNodes())
    {
        FString tag = node->GetTag();

        if (tag.Equals(TEXT("color")))
        {
            materialSpecification->Color =
                this->ParseFVector4FromAttributeString(
                    node->GetAttribute(TEXT("rgba")));
        }
        else if (tag.Equals(TEXT("unreal_material")))
        {
            materialSpecification->TextureFile =
                node->GetAttribute(TEXT("path"));
        }
    }

    if (!hasName)
    {
        throw std::runtime_error("Material encountered without a name.");
    }

    return materialSpecification;
}

UrdfLinkInertialSpecification*
UrdfParser::ParseLinkInertialSpecification(FXmlNode* inertialNode)
{
    UrdfLinkInertialSpecification* inertialSpecification =
        new UrdfLinkInertialSpecification();

    bool hasOriginNode = false;
    bool hasMassNode = false;
    bool hasInertiaNode = false;

    for (auto node : inertialNode->GetChildrenNodes())
    {
        FString tag = node->GetTag();

        if (tag.Equals(TEXT("origin")))
        {
            if (hasOriginNode)
                throw std::runtime_error(
                    "Multiple origin nodes in inertial specification.");

            hasOriginNode = true;
            inertialSpecification->Origin = this->ParseUrdfOrigin(node);
        }
        else if (tag.Equals(TEXT("mass")))
        {
            if (hasMassNode)
                throw std::runtime_error(
                    "Multiple mass nodes in inertial specification.");

            hasMassNode = true;
            inertialSpecification->Mass =
                FCString::Atof(*node->GetAttribute(TEXT("value")));
        }
        else if (tag.Equals(TEXT("inertia")))
        {
            if (hasInertiaNode)
                throw std::runtime_error(
                    "Multiple inertia nodes in inertial specification.");

            hasInertiaNode = true;
            inertialSpecification->Inertia =
                this->ParseInertiaMatrixFromAttributes(node);
        }
    }

    if (!hasInertiaNode)
        throw std::runtime_error(
            "No inertia node specified in inertial specification.");
    if (!hasMassNode)
        throw std::runtime_error(
            "No mass node specified in inertial specification.");

    return inertialSpecification;
}

UrdfLinkVisualSpecification*
UrdfParser::ParseLinkVisualSpecification(FXmlNode* visualNode)
{
    UrdfLinkVisualSpecification* visualSpecification =
        new UrdfLinkVisualSpecification();

    bool hasNameAttribute = false;
    bool hasOriginNode = false;
    bool hasGeometryNode = false;

    for (auto attr : visualNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("name")))
        {
            if (hasNameAttribute)
            {
                throw std::runtime_error(
                    "Multiple name attributes detected for visual node. First "
                    "name is '" +
                    std::string(TCHAR_TO_UTF8(*visualSpecification->Name)) +
                    "'.");
            }

            hasNameAttribute = true;
            visualSpecification->Name = attr.GetValue();
        }
    }

    for (auto node : visualNode->GetChildrenNodes())
    {
        FString tag = node->GetTag();

        if (tag.Equals(TEXT("origin")))
        {
            if (hasOriginNode)
            {
                if (hasNameAttribute)
                    throw std::runtime_error(
                        "Multiple origin nodes detected for visual node with "
                        "name '" +
                        std::string(TCHAR_TO_UTF8(*visualSpecification->Name)) +
                        "'.");
                else
                    throw std::runtime_error("Multiple origin nodes detected "
                                             "for unnamed visual node.");
            }

            hasOriginNode = true;
            visualSpecification->Origin = this->ParseUrdfOrigin(node);
        }
        else if (tag.Equals(TEXT("geometry")))
        {
            if (hasGeometryNode)
            {
                if (hasNameAttribute)
                    throw std::runtime_error(
                        "Multiple geometry nodes detected for visual node with "
                        "name '" +
                        std::string(TCHAR_TO_UTF8(*visualSpecification->Name)) +
                        "'.");
                else
                    throw std::runtime_error("Multiple geometry nodes detected "
                                             "for unnamed visual node.");
            }

            hasGeometryNode = true;
            visualSpecification->Geometry =
                this->ParseUrdfGeometrySpecification(node);
        }
        else if (tag.Equals(TEXT("material")))
        {
            visualSpecification->MaterialName =
                node->GetAttribute(TEXT("name"));
        }
    }

    if (!hasGeometryNode)
    {
        if (hasNameAttribute)
            throw std::runtime_error(
                "No geometry specified for visual node with name '" +
                std::string(TCHAR_TO_UTF8(*visualSpecification->Name)) + "'.");
        else
            throw std::runtime_error(
                "No geometry specified for unnamed visual node.");
    }

    return visualSpecification;
}

UrdfLinkCollisionSpecification*
UrdfParser::ParseLinkCollisionSpecification(FXmlNode* collisionNode)
{
    UrdfLinkCollisionSpecification* collisionSpecification =
        new UrdfLinkCollisionSpecification();

    bool hasGeometryNode = false;
    bool hasOriginNode = false;
    bool hasNameAttribute = false;

    for (auto attr : collisionNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("name")))
        {
            if (hasNameAttribute)
            {
                throw std::runtime_error(
                    "Multiple name attributes detected for collision node. "
                    "First name is '" +
                    std::string(TCHAR_TO_UTF8(*collisionSpecification->Name)) +
                    "'.");
            }

            hasNameAttribute = true;
            collisionSpecification->Name = attr.GetValue();
        }
    }

    for (auto node : collisionNode->GetChildrenNodes())
    {
        FString tag = node->GetTag();

        if (tag.Equals(TEXT("origin")))
        {
            if (hasOriginNode)
            {
                if (hasNameAttribute)
                    throw std::runtime_error(
                        "Multiple origin nodes detected for collision node "
                        "with name '" +
                        std::string(
                            TCHAR_TO_UTF8(*collisionSpecification->Name)) +
                        "'.");
                else
                    throw std::runtime_error(
                        "Multiple origin nodes on unnamed collision node.");
            }

            hasOriginNode = true;
            collisionSpecification->Origin = this->ParseUrdfOrigin(node);
        }
        else if (tag.Equals(TEXT("geometry")))
        {
            if (hasGeometryNode)
            {
                if (hasNameAttribute)
                    throw std::runtime_error(
                        "Multiple geometry nodes detected for collision node "
                        "with name '" +
                        std::string(
                            TCHAR_TO_UTF8(*collisionSpecification->Name)) +
                        "'.");
                else
                    throw std::runtime_error("Multiple geometry elements on "
                                             "unnamed collision node.");
            }

            hasGeometryNode = true;
            collisionSpecification->Geometry =
                this->ParseUrdfGeometrySpecification(node);
        }
    }

    if (!hasGeometryNode)
    {
        if (hasNameAttribute)
            throw std::runtime_error(
                "No geometry node specified for collision node " +
                std::string(TCHAR_TO_UTF8(*collisionSpecification->Name)) +
                ".");
        else
            throw std::runtime_error(
                "No geometry node specified for unnamed collision node.");
    }

    return collisionSpecification;
}

UrdfGeometry* UrdfParser::ParseUrdfGeometrySpecification(FXmlNode* geometryNode)
{
    auto childNodes = geometryNode->GetChildrenNodes();

    if (childNodes.Num() != 1)
        throw std::runtime_error("Invalid number of child nodes to geometry "
                                 "element in link. Must be 1, but found " +
                                 std::to_string(childNodes.Num()) + ".");

    UrdfGeometry* result = nullptr;

    for (auto node : childNodes)
    {
        FString tag = node->GetTag();

        if (tag.Equals(TEXT("box")))
        {
            auto box = new UrdfBox();
            box->Size =
                ParseFVectorFromAttributeString(node->GetAttribute("size"));
            result = box;
        }
        else if (tag.Equals(TEXT("cylinder")))
        {
            auto cylinder = new UrdfCylinder();
            cylinder->Length = FCString::Atof(*(node->GetAttribute("length")));
            cylinder->Radius = FCString::Atof(*(node->GetAttribute("radius")));
            result = cylinder;
        }
        else if (tag.Equals(TEXT("sphere")))
        {
            auto sphere = new UrdfSphere();
            sphere->Radius = FCString::Atof(*(node->GetAttribute("radius")));
            result = sphere;
        }
        else if (tag.Equals(TEXT("mesh")))
        {
            auto mesh = new UrdfMesh();

            bool foundLocation = false;
            bool foundType = false;
            bool foundReverseNormals = false;
            bool foundScaleFactor = false;
            bool foundDynamicCollisionType = false;
            bool foundVhacdConcavity = false;
            bool foundVhacdResolution = false;
            bool foundVhacdMasNumVerticesPerCh = false;
            bool voundVhacdMinVolumePerCh = false;
            bool foundVhacdOutputFolderPath = false;

            for (auto kvp : node->GetAttributes())
            {
                if (kvp.GetTag().Equals(TEXT("location")))
                {
                    if (foundLocation)
                    {
                        throw std::runtime_error(
                            "Multiple locations specified for geometry.");
                    }
                    // mesh->FileLocation =
                    // RobotSim::Settings::getPorjectDirectoryFullPath(TCHAR_TO_UTF8(*kvp.GetValue())).c_str();
                    if (kvp.GetValue().StartsWith(TEXT("StaticMesh")))
                    {
                        mesh->FileLocation = kvp.GetValue();
                    }
                    else
                    {
                        mesh->FileLocation =
                            RobotSim::Settings::getPluginDirectoryFullPath(
                                TCHAR_TO_UTF8(*kvp.GetValue()))
                                .c_str();
                    }
                    foundLocation = true;
                }
                else if (kvp.GetTag().Equals(TEXT("type")))
                {
                    if (foundType)
                    {
                        throw std::runtime_error(
                            "Multiple file types specified for geometry.");
                    }

                    auto value = kvp.GetValue();
                    if (value.Equals(TEXT("stl_ascii")))
                    {
                        mesh->FileType = STL_ASCII;
                    }
                    else if (value.Equals(TEXT("unreal_mesh")))
                    {
                        mesh->FileType = UNREAL_MESH;
                    }
                    else
                    {
                        throw std::runtime_error(
                            "Unrecognized file type: " +
                            std::string(TCHAR_TO_UTF8(*(value))) +
                            ". Valid values are 'stl_ascii', 'unreal_mesh'");
                    }

                    foundType = true;
                }
                else if (kvp.GetTag().Equals(TEXT("reverse_normals")))
                {
                    if (foundReverseNormals)
                    {
                        throw std::runtime_error(
                            "Multiple instances of reverse_normals specified "
                            "for geometry.");
                    }

                    auto value = kvp.GetValue();
                    if (value.Equals(TEXT("true")))
                    {
                        mesh->ReverseNormals = true;
                    }

                    foundReverseNormals = true;
                }
                else if (kvp.GetTag().Equals(TEXT("scale_factor")))
                {
                    if (foundScaleFactor)
                    {
                        throw std::runtime_error(
                            "Multiple instances of scale_factor specified for "
                            "geometry.");
                    }

                    mesh->ScaleFactor = FCString::Atof(*kvp.GetValue());
                }
                else if (kvp.GetTag().Equals(TEXT("vhacd_concavity")))
                {
                    if (foundVhacdConcavity)
                    {
                        throw std::runtime_error(
                            "Multiple instances of vhacd_concavity specified "
                            "for geometry.");
                    }

                    mesh->VhacdConcavity = FCString::Atof(*kvp.GetValue());
                }
                else if (kvp.GetTag().Equals(TEXT("vhacd_resolution")))
                {
                    if (foundVhacdResolution)
                    {
                        throw std::runtime_error(
                            "Multiple instances of vhacd_resolution specified "
                            "for geometry.");
                    }

                    mesh->VhacdResolution = FCString::Atoi(*kvp.GetValue());
                }
                else if (kvp.GetTag().Equals(
                             TEXT("vhacd_max_num_vertices_per_ch")))
                {
                    if (foundVhacdMasNumVerticesPerCh)
                    {
                        throw std::runtime_error("Multiple instances of "
                                                 "vhacd_max_num_vertices_per_"
                                                 "ch specified for geometry.");
                    }

                    mesh->VhacdMaxNumVerticesPerCh =
                        FCString::Atoi(*kvp.GetValue());
                }
                else if (kvp.GetTag().Equals(TEXT("vhacd_min_volume_per_ch")))
                {
                    if (foundVhacdConcavity)
                    {
                        throw std::runtime_error(
                            "Multiple instances of vhacd_min_volume_per_ch "
                            "specified for geometry.");
                    }

                    mesh->VhacdMinVolumePerCh = FCString::Atof(*kvp.GetValue());
                }
                else if (kvp.GetTag().Equals(TEXT("vhacd_output_folder_path")))
                {
                    if (foundVhacdConcavity)
                    {
                        throw std::runtime_error(
                            "Multiple instances of vhacd_output_folder_path "
                            "specified for geometry.");
                    }

                    mesh->VhacdOutputFolderPath = kvp.GetValue();
                }
                else if (kvp.GetTag().Equals(TEXT("dynamic_collision_type")))
                {
                    if (foundDynamicCollisionType)
                    {
                        throw std::runtime_error(
                            "Multiple instances of dynamic_collision_type "
                            "specified for geometry.");
                    }

                    FString collisionType = *kvp.GetValue();

                    // TODO: does this check case?
                    if (collisionType.Equals(TEXT("bsp"),
                                             ESearchCase::IgnoreCase))
                    {
                        mesh->DynamicCollisionType = COL_BSP;
                    }
                    else if (collisionType.Equals(TEXT("vhacd"),
                                                  ESearchCase::IgnoreCase))
                    {
                        mesh->DynamicCollisionType = COL_VHACD;
                    }
                    else if (collisionType.Equals(TEXT("manual"),
                                                  ESearchCase::IgnoreCase))
                    {
                        mesh->DynamicCollisionType = COL_MANUAL;
                    }
                    else if (collisionType.Equals(TEXT("box"),
                                                  ESearchCase::IgnoreCase))
                    {
                        mesh->DynamicCollisionType = COL_BOX;
                    }
                    else if (collisionType.Equals(TEXT("cyliner"),
                                                  ESearchCase::IgnoreCase))
                    {
                        mesh->DynamicCollisionType = COL_CYLINER;
                    }
                    else if (collisionType.Equals(TEXT("sphere"),
                                                  ESearchCase::IgnoreCase))
                    {
                        mesh->DynamicCollisionType = COL_SPHERE;
                    }
                    else
                    {
                        throw std::runtime_error(
                            "Invalid collision type specified: '" +
                            std::string(TCHAR_TO_UTF8(*collisionType)) +
                            "'. Valid values are 'bsp', 'vhacd', and "
                            "'manual.'");
                    }
                }
            }

            if (!foundLocation)
                throw std::runtime_error("No location specified for mesh.");
            if (!foundType)
                throw std::runtime_error("No type specified for mesh.");

            result = mesh;
        }
    }

    if (result == nullptr)
        throw std::runtime_error(
            "Could not parse geometry element : type was not one of 'box', "
            "'cylinder', 'sphere', or 'mesh'");

    return result;
}

UrdfJointCalibrationSpecification*
UrdfParser::ParseJointCalibrationSpecification(FXmlNode* calibrationNode)
{
    auto calibrationSpecification = new UrdfJointCalibrationSpecification();

    for (auto attr : calibrationNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("rising")))
            calibrationSpecification->Rising = FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("falling")))
            calibrationSpecification->Falling =
                FCString::Atof(*attr.GetValue());
    }

    return calibrationSpecification;
}

UrdfJointDynamicsSpecification
UrdfParser::ParseJointDynamicsSpecification(FXmlNode* dynamicsNode)
{
    UrdfJointDynamicsSpecification dynamicsSpecification;

    for (auto attr : dynamicsNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("damping")))
            dynamicsSpecification.Damping = FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("friction")))
            dynamicsSpecification.Friction = FCString::Atod(*attr.GetValue());
    }

    return dynamicsSpecification;
}

UrdfJointLimitSpecification*
UrdfParser::ParseJointLimitSpecification(FXmlNode* limitNode)
{
    auto limitSpecification = new UrdfJointLimitSpecification();

    bool hasEffort = false;
    bool hasVelocity = false;

    for (auto attr : limitNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("lower")))
            limitSpecification->Lower = FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("upper")))
            limitSpecification->Upper = FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("velocity")))
        {
            hasVelocity = true;
            limitSpecification->Velocity = FCString::Atof(*attr.GetValue());
        }
        else if (tag.Equals(TEXT("effort")))
        {
            hasEffort = true;
            limitSpecification->Effort = FCString::Atof(*attr.GetValue());
        }
    }

    if (!hasEffort)
        throw std::runtime_error("Missing effort in limit specification.");
    if (!hasVelocity)
        throw std::runtime_error("Missing velocity in limit specification.");

    return limitSpecification;
}

UrdfJointMimicSpecification*
UrdfParser::ParseJointMimicSpecification(FXmlNode* mimicNode)
{
    auto mimicSpecification = new UrdfJointMimicSpecification();

    bool hasJointName = false;

    for (auto attr : mimicNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("multiplier")))
            mimicSpecification->Multiplier = FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("offset")))
            mimicSpecification->Offset = FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("joint")))
        {
            hasJointName = true;
            mimicSpecification->JointName = attr.GetValue();
        }
    }

    if (!hasJointName)
        throw std::runtime_error("Missing joint name in mimic specification");

    return mimicSpecification;
}

UrdfJointSafetyControllerSpecification*
UrdfParser::ParseJointSafetyControllerSpecification(
    FXmlNode* safetyControllerNode)
{
    auto safetyControllerSpecification =
        new UrdfJointSafetyControllerSpecification();

    bool hasKVelocity = false;

    for (auto attr : safetyControllerNode->GetAttributes())
    {
        FString tag = attr.GetTag();
        if (tag.Equals(TEXT("soft_lower_limit")))
            safetyControllerSpecification->SoftLowerLimit =
                FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("soft_upper_limit")))
            safetyControllerSpecification->SoftUpperLimit =
                FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("k_position")))
            safetyControllerSpecification->KPosition =
                FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("k_velocity")))
        {
            safetyControllerSpecification->KVelocity =
                FCString::Atof(*attr.GetValue());
            hasKVelocity = true;
        }
    }

    if (!hasKVelocity)
        throw std::runtime_error(
            "Missing k_velocity element on Safety Controller node.");

    return safetyControllerSpecification;
}

UrdfAngularForceSpecification*
UrdfParser::ParseAngularForceSpecification(FXmlNode* angularForceNode)
{
    UrdfAngularForceSpecification* forceSpecification =
        new UrdfAngularForceSpecification();

    bool isLinkNameSpecified = false;
    bool isAxisSpecified = false;

    for (auto attr : angularForceNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("link_name")))
        {
            if (isLinkNameSpecified)
            {
                throw std::runtime_error(
                    "Multiple Link Names specified for angular force.");
            }

            isLinkNameSpecified = true;
            forceSpecification->LinkName = attr.GetValue();
        }
        else if (tag.Equals(TEXT("axis")))
        {
            if (isAxisSpecified)
            {
                throw std::runtime_error(
                    "Multiple axis specified for angular force.");
            }

            isAxisSpecified = true;
            forceSpecification->Axis =
                this->ParseFVectorFromAttributeString(attr.GetValue());
        }
    }

    if (!isLinkNameSpecified)
        throw std::runtime_error("No link name specified for angular force.");

    if (!isAxisSpecified)
        throw std::runtime_error("No axis specified for angular force.");

    return forceSpecification;
}

UrdfLinearForceSpecification*
UrdfParser::ParseLinearForceSpecification(FXmlNode* linearForceNode)
{
    UrdfLinearForceSpecification* forceSpecification =
        new UrdfLinearForceSpecification();

    bool isLinkNameSpecified = false;
    bool isAxisSpecified = false;
    bool isApplicationPointSpecified = false;

    for (auto attr : linearForceNode->GetAttributes())
    {
        FString tag = attr.GetTag();

        if (tag.Equals(TEXT("link_name")))
        {
            if (isLinkNameSpecified)
            {
                throw std::runtime_error(
                    "Multiple Link Names specified for linear force.");
            }

            isLinkNameSpecified = true;
            forceSpecification->LinkName = attr.GetValue();
        }
        else if (tag.Equals(TEXT("axis")))
        {
            if (isAxisSpecified)
            {
                throw std::runtime_error(
                    "Multiple axis specified for linear force.");
            }

            isAxisSpecified = true;
            forceSpecification->Axis =
                this->ParseFVectorFromAttributeString(attr.GetValue());
        }
        else if (tag.Equals(TEXT("application_point")))
        {
            if (isApplicationPointSpecified)
            {
                throw std::runtime_error(
                    "Multiple application points specified for linear force.");
            }

            isApplicationPointSpecified = true;
            forceSpecification->ApplicationPoint =
                this->ParseFVectorFromAttributeString(attr.GetValue());
        }
    }

    if (!isLinkNameSpecified)
        throw std::runtime_error("No link name specified for linear force.");
    if (!isAxisSpecified)
        throw std::runtime_error("No axis specified for linear force.");
    if (!isApplicationPointSpecified)
        throw std::runtime_error(
            "No application point specified for linear force.");

    return forceSpecification;
}

UrdfOrigin UrdfParser::ParseUrdfOrigin(FXmlNode* originNode)
{
    UrdfOrigin origin;

    for (auto attr : originNode->GetAttributes())
    {
        FString tag = attr.GetTag();
        if (tag.Equals(TEXT("xyz")))
            origin.Origin = ParseFVectorFromAttributeString(attr.GetValue());
        else if (tag.Equals(TEXT("rpy")))
            origin.RollPitchYaw =
                ParseFRotatorFromAttributeString(attr.GetValue());
    }

    return origin;
}

FVector UrdfParser::ParseFVectorFromAttributeString(const FString& string)
{
    TArray<FString> splitString;
    string.ParseIntoArray(splitString, TEXT(" "), true);
    if (splitString.Num() == 0)
    {
        return FVector(0.0, 0.0, 0.0);
    }
    if (splitString.Num() != 3)
    {
        throw std::runtime_error("Error parsing 3-d vector from string " +
                                 std::string(TCHAR_TO_UTF8(*string)) +
                                 ", did not detect 3 elements.");
    }

    return FVector(FCString::Atof(*splitString[0]),
                   FCString::Atof(*splitString[1]),
                   FCString::Atof(*splitString[2]));
}

FRotator UrdfParser::ParseFRotatorFromAttributeString(const FString& string)
{
    TArray<FString> splitString;
    string.ParseIntoArray(splitString, TEXT(" "), true);
    if (splitString.Num() == 0)
    {
        return FRotator(0.0, 0.0, 0.0);
    }
    if (splitString.Num() != 3)
    {
        throw std::runtime_error("Error parsing 3-d vector from string " +
                                 std::string(TCHAR_TO_UTF8(*string)) +
                                 ", did not detect 3 elements.");
    }

    return FRotator(FCString::Atof(*splitString[1]),
                    FCString::Atof(*splitString[2]),
                    FCString::Atof(*splitString[0]));
}

FVector4 UrdfParser::ParseFVector4FromAttributeString(const FString& string)
{
    TArray<FString> splitString;
    string.ParseIntoArray(splitString, TEXT(" "), true);

    if (splitString.Num() != 4)
    {
        throw std::runtime_error("Error parsing 4-d vector from string " +
                                 std::string(TCHAR_TO_UTF8(*string)) +
                                 ", did not detect 4 elements.");
    }

    return FVector4(
        FCString::Atof(*splitString[0]), FCString::Atof(*splitString[1]),
        FCString::Atof(*splitString[2]), FCString::Atof(*splitString[3]));
}

RobotSim::VectorMathf::Matrix3x3f
UrdfParser::ParseInertiaMatrixFromAttributes(FXmlNode* inertiaNode)
{
    RobotSim::VectorMathf::Matrix3x3f inertiaMatrix;

    float ixx = std::numeric_limits<float>::quiet_NaN();
    float ixy = std::numeric_limits<float>::quiet_NaN();
    float ixz = std::numeric_limits<float>::quiet_NaN();
    float iyy = std::numeric_limits<float>::quiet_NaN();
    float iyz = std::numeric_limits<float>::quiet_NaN();
    float izz = std::numeric_limits<float>::quiet_NaN();

    for (auto attr : inertiaNode->GetAttributes())
    {
        FString tag = attr.GetTag();
        if (tag.Equals(TEXT("ixx")))
            ixx = FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("ixy")))
            ixy = FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("ixz")))
            ixz = FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("iyy")))
            iyy = FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("iyz")))
            iyz = FCString::Atof(*attr.GetValue());
        else if (tag.Equals(TEXT("izz")))
            izz = FCString::Atof(*attr.GetValue());
    }

    if (std::isnan(ixx))
        throw std::runtime_error(
            "Error parsing inertia node: ixx cannot be parsed.");
    else if (std::isnan(ixy))
        throw std::runtime_error(
            "Error parsing inertia node: ixy cannot be parsed.");
    else if (std::isnan(ixz))
        throw std::runtime_error(
            "Error parsing inertia node: ixz cannot be parsed.");
    else if (std::isnan(iyy))
        throw std::runtime_error(
            "Error parsing inertia node: iyy cannot be parsed.");
    else if (std::isnan(iyz))
        throw std::runtime_error(
            "Error parsing inertia node: iyz cannot be parsed.");
    else if (std::isnan(izz))
        throw std::runtime_error(
            "Error parsing inertia node: izz cannot be parsed.");

    int x = 0;
    int y = 1;
    int z = 2;

    inertiaMatrix(x, x) = ixx;
    inertiaMatrix(x, y) = ixy;
    inertiaMatrix(x, z) = ixz;
    inertiaMatrix(y, x) = ixy;
    inertiaMatrix(y, y) = iyy;
    inertiaMatrix(y, z) = iyz;
    inertiaMatrix(z, x) = ixz;
    inertiaMatrix(z, y) = iyz;
    inertiaMatrix(z, z) = izz;

    return inertiaMatrix;
}

UrdfGeometryType UrdfParser::GeometryTypeFromString(FString typeString)
{
    typeString.ToLowerInline();

    if (typeString.Equals(TEXT("box")))
        return BOX;
    else if (typeString.Equals(TEXT("cylinder")))
        return CYLINDER;
    else if (typeString.Equals(TEXT("sphere")))
        return SPHERE;
    else if (typeString.Equals(TEXT("mesh")))
        return MESH;

    throw std::runtime_error(
        "Unrecognized geometry type '" +
        std::string(TCHAR_TO_UTF8(*typeString)) +
        ". Valid options are 'box', 'cylinder', 'sphere', and 'mesh'.");
}