#include "SphereAgentController.h"

//remove
#include <iostream>
//this

#include <map>
#include <string>
#include <vector>

#include "Box.h"

SphereAgentController::SphereAgentController(UWorld* world)
{
    for (TActorIterator<AActor> ActorItr(world, AActor::StaticClass()); ActorItr; ++ActorItr)
    {
        UE_LOG(LogTemp, Warning, TEXT("%s"), *(*ActorItr)->GetName());
        if ((*ActorItr)->GetName().Equals(TEXT("SphereAgent"), ESearchCase::IgnoreCase))
        { 
            UE_LOG(LogTemp, Warning, TEXT("Sphere actor found!"));
            sphere_agent_ = (*ActorItr);
            break;
        }
    }
}

void SphereAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    std::cout << "-------------------------------------Received actions!-----------------------------------------" << std::endl;
    for (auto a : action)
    {
        std::cout << "Name: " << a.first << ", action: ";
        for(auto b: a.second)
        {
            std::cout << b << ", ";
        }
        std::cout << std::endl;
    }
    
    std::vector<float> action_vec = action.at("set_location");

    FVector Location = FVector(action_vec.at(0), action_vec.at(1), 0.0f);
    sphere_agent_->SetActorLocation(Location);
}

std::map<std::string, std::vector<uint8_t>> SphereAgentController::getObservation()
{
    FVector sphere_agent_location = sphere_agent_->GetActorLocation();

    std::vector<float> src = {sphere_agent_location.X, sphere_agent_location.Y, sphere_agent_location.Z};
    std::map<std::string, std::vector<uint8_t>> return_data;

    return_data.emplace("location", std::vector<uint8_t>{});

    return_data["location"] = serializeToUint8(src);

    return return_data;
}

std::map<std::string, Box> SphereAgentController::getObservationSpace()
{
    std::map<std::string, Box> return_data;
    return_data.emplace("location", Box());

    return_data["location"].low = std::numeric_limits<float>::lowest();
    return_data["location"].high = std::numeric_limits<float>::max();
    return_data["location"].shape = {3};
    return_data["location"].dtype = DataType::Float32;
    return return_data;
}

std::map<std::string, Box> SphereAgentController::getActionSpace()
{
    std::map<std::string, Box> return_data;
    return_data.emplace("set_location", Box());

    return_data["set_location"].low = std::numeric_limits<float>::lowest();
    return_data["set_location"].high = std::numeric_limits<float>::max();
    return_data["set_location"].shape = {3};
    return_data["set_location"].dtype = DataType::Float32;

    return return_data;
}
