#pragma once

class CameraSensor;
class UWorld;

class DepthCameraSensor : public CameraSensor
{
public:
    DepthCameraSensor(UWorld* world);
    ~DepthCameraSensor();

    //Add GetDepthRenderSource std::vector<uint8_t>
};