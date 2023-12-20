//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

class AActor;
class UWorld;

class Visualizer
{
public:
    Visualizer() = delete;
    Visualizer(UWorld* world);
    ~Visualizer();

    void findObjectReferences(UWorld* world);
    void cleanUpObjectReferences();

private:
    AActor* actor_ = nullptr;
};
