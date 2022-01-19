
#pragma once

#include <utility>
#include <vector>

#include "ActionSpec.h"

namespace unrealrl
{
/**
 * Contains multiple @see ActionSpec.h
 */
class ActionSpecs
{
public:
    ActionSpecs()
    {
    }

    ActionSpecs(const ActionSpecs& src)
    {
        NumActions = src.NumActions;
        Specs = src.Specs;
    }

    ActionSpecs(const ActionSpec& Spec)
    {
        NumActions = 1;
        Specs.resize(NumActions);
        Specs[0] = Spec;
    }

    ActionSpecs(const std::vector<ActionSpec>& SpecsVec)
    {
        NumActions = SpecsVec.size();
        Specs = SpecsVec;
    }

    ActionSpecs(size_t tNumActions)
    {
        NumActions = tNumActions;
        Specs.resize(NumActions);
    }

    void Append(const ActionSpec& Spec)
    {
        NumActions++;
        Specs.push_back(Spec);
    }

    MSGPACK_DEFINE_MAP(NumActions, Specs);

    size_t NumActions = 0;
    std::vector<ActionSpec> Specs;
};
} // namespace unrealrl
