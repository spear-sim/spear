#pragma once

class AActor;
class UWorld;

class Visualizer
{
public:
    Visualizer(UWorld* world);
    ~Visualizer();

    void findObjectReferences(UWorld* world);
    void cleanUpObjectReferences();

private:
    AActor* actor_ = nullptr;
};
