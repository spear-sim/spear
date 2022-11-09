#pragma once

class ACameraActor;
class UWorld;

class Visualizer
{
public:
    Visualizer(UWorld* world);
    ~Visualizer();

    void findObjectReferences(UWorld* world);
    void cleanUpObjectReferences();

private:
    ACameraActor* camera_actor_ = nullptr;
};
