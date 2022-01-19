#pragma once

#include <map>
#include "Action.h"

namespace unrealrl
{
class ActionDict
{
public:
    ActionDict()
    {
    }

    ActionDict(const ActionDict& src)
    {
        AgentToActionMap = src.AgentToActionMap;
    }

    FORCEINLINE std::map<std::string, std::vector<Action>>*
    GetAgentToActionMap()
    {
        return &AgentToActionMap;
    }

    MSGPACK_DEFINE_MAP(AgentToActionMap);

private:
    std::map<std::string, std::vector<Action>> AgentToActionMap;
};

} // namespace unrealrl
