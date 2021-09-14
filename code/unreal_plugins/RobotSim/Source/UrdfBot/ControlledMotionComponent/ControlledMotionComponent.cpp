#include "ControlledMotionComponent.h"
#include "UrdfBot/UrdfLink.h"

void ControlledMotionComponent::Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent)
{
	this->baseLink_ = baseLink;
	this->actuationLink_ = actuationLink;
	this->worldScale = URobotBlueprintLib::GetWorldToMetersScale(baseLink);
	this->jointSpecification_ = jointSpecification;
	this->constraintComponent_ = constraintComponent;
	this->name_ = jointSpecification->Name;
}

UPhysicsConstraintComponent* ControlledMotionComponent::GetConstraintComponent()
{
	return this->constraintComponent_;
}

AUrdfLink* ControlledMotionComponent::GetParentLink()
{
	return this->baseLink_;
}

AUrdfLink* ControlledMotionComponent::GetActuatorLink()
{
	return this->actuationLink_;
}

bool ControlledMotionComponent::GetFloatFromConfiguration(FString configName, TMap<FString, FString> configuration, float& out, bool throwIfNotExist, bool throwIfNonNumeric)
{
    if (!configuration.Contains(configName))
    {
        if (throwIfNotExist)
            throw std::runtime_error("Required configuration component '" + std::string(TCHAR_TO_UTF8(*configName)) + "' is missing from ControlledMotionComponent '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) + "'.");
        out = 0.0;
        return false;
    }

    FString configValue = configuration[configName];
    if (!configValue.IsNumeric())
    {
        if (throwIfNonNumeric)
            throw std::runtime_error("Required configuration component '" + std::string(TCHAR_TO_UTF8(*configName)) + "' for ControlledMotionComponent '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) + "' has the value '" + std::string(TCHAR_TO_UTF8(*configValue)) + "', which is not numeric.");
        out = 0.0;
        return false;
    }
    out = FCString::Atof(*configValue);
    return true;
}