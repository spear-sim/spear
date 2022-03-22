#include "SphereAgentController.h"

//remove
#include <iostream>
//this

#include <map>
#include <string>
#include <vector>

#include "Types.h"

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
}

std::map<std::string, std::vector<uint8_t>> SphereAgentController::getObservation()
{
    std::vector<float> src = {1, 2, 3, 4};
    std::map<std::string, std::vector<uint8_t>> return_data;

    return_data.emplace("vector", std::vector<uint8_t>{});

    serializeToUint8(src, return_data["vector"]);

    return return_data;
}

std::map<std::string, Box> SphereAgentController::getObservationSpace()
{
    std::map<std::string, Box> return_data;
    return_data.emplace("vector", Box());

    return_data["vector"].low = 0;
    return_data["vector"].high = 1;
    return_data["vector"].shape = {4u};
    return_data["vector"].dtype = DataType::Float32;
    return return_data;
}

std::map<std::string, Box> SphereAgentController::getActionSpace()
{
    std::map<std::string, Box> return_data;
    return_data.emplace("force", Box());

    return_data["force"].low = 0;
    return_data["force"].high = 1;
    return_data["force"].shape = {4u};
    return_data["force"].dtype = DataType::Float32;
    return return_data;
}
