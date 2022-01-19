#pragma once

#include <vector>

#include "Utils/Types.h"
#include "ObservationSpec.h"

/**
 * Contains multiple @see ObservationSpec.h
 */

namespace unrealrl
{
class ObservationSpecs
{
public:
    // We need default constructor for MSGPACK binding
    ObservationSpecs()
    {
    }

    ObservationSpecs(const ObservationSpecs& src)
    {
        NumObservations = src.NumObservations;
        Specs = src.Specs;
    }

    ObservationSpecs(const ObservationSpec& Spec)
    {
        NumObservations = 1;
        Specs.resize(NumObservations);
        Specs[0] = Spec;
    }

    ObservationSpecs(const std::vector<ObservationSpec>& SpecsVec)
    {
        NumObservations = SpecsVec.size();
        Specs = SpecsVec;
    }

    ObservationSpecs(size_t tNumObservations)
    {
        NumObservations = tNumObservations;
        Specs.resize(NumObservations);
    }

    void Append(const ObservationSpec& Spec)
    {
        NumObservations++;
        Specs.push_back(Spec);
    }

    MSGPACK_DEFINE_MAP(NumObservations, Specs);

    size_t NumObservations = 0;
    std::vector<ObservationSpec> Specs;
};
} // namespace unrealrl
