#pragma once

class CameraSensor;
class UWorld;

class SegmentationCameraSensor : public CameraSensor
{
public:
    SegmentationCameraSensor(UWorld* world);
    ~SegmentationCameraSensor();

    //Add GetSegmentationRenderSource std::vector<uint8_t>
};