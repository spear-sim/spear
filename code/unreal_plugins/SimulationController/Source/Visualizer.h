#pragma once

class AActor;
class UWorld;

class Visualizer
{
public:
    Visualizer() = default;
    ~Visualizer() = default;

    void findObjectReferences(UWorld* world);
    void cleanUpObjectReferences();

private:
    AActor* camera_actor_ = nullptr;
};
