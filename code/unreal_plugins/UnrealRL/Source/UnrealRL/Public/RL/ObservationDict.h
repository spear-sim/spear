#pragma once

#include <map>

#include "Observation.h"

namespace unrealrl
{
/**
 * Used to communicate all agent's observations to the client.
 * Maps an agent with its observation.
 */
class ObservationDict
{
public:
    ObservationDict()
    {
    }

    ObservationDict(const ObservationDict& src)
    {
        AgentToObservationMap = src.AgentToObservationMap;
    }

    FORCEINLINE std::map<std::string, std::vector<Observation>>*
    GetAgentToObservationMap()
    {
        return &AgentToObservationMap;
    }

    MSGPACK_DEFINE_MAP(AgentToObservationMap);

    FORCEINLINE friend std::ostream& operator<<(std::ostream& os,
                                                ObservationDict* s)
    {
        os << "Received " << s->AgentToObservationMap.size()
           << " Agents' observations."
           << "\n";

        for (auto it = std::begin(s->AgentToObservationMap);
             it != std::end(s->AgentToObservationMap); ++it)
        {
            os << "AgentID " << it->first << "\n";
        }
        return os;
    }

private:
    std::map<std::string, std::vector<Observation>> AgentToObservationMap;
};

} // namespace unrealrl
