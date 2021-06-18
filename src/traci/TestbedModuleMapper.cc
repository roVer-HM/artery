#include "traci/TestbedModuleMapper.h"
#include <omnetpp/ccomponenttype.h>

namespace traci
{

Define_Module(TestbedModuleMapper)

void TestbedModuleMapper::initialize()
{
    m_twinId = par("twinId").stringValue();
    m_twinType = omnetpp::cModuleType::get(par("twinType"));
    BasicModuleMapper::initialize();
}

omnetpp::cModuleType* TestbedModuleMapper::vehicle(NodeManager& manager, const std::string& id)
{
    return m_twinId == id ? m_twinType : BasicModuleMapper::vehicle(manager, id);
}

omnetpp::cModuleType* TestbedModuleMapper::person(NodeManager& manager, const std::string& id)
{
    return m_twinId == id ? m_twinType : BasicModuleMapper::person(manager, id);
}

} /* namespace traci */
