#include "traci/BasicNodeManager.h"
#include "traci/CheckTimeSync.h"
#include "traci/Core.h"
#include "traci/LiteAPI.h"
#include "traci/ModuleMapper.h"
#include "traci/VariableCache.h"
#include "traci/MovingObjectSink.h"
#include <inet/common/ModuleAccess.h>

using namespace omnetpp;

namespace traci
{
namespace
{
static const std::set<int> sVehicleVariables {
    VAR_POSITION, VAR_SPEED, VAR_ANGLE
};
static const std::set<int> sSimulationVariables {
    VAR_DEPARTED_VEHICLES_IDS, VAR_ARRIVED_VEHICLES_IDS, VAR_TIME
};

class VehicleObjectImpl : public BasicNodeManager::VehicleObject
{
public:
    VehicleObjectImpl(std::shared_ptr<VehicleCache> cache) : m_cache(cache) {}

    std::shared_ptr<VehicleCache> getCache() const override { return m_cache; }
    const TraCIPosition& getPosition() const override { return m_cache->get<VAR_POSITION>(); }
    TraCIAngle getHeading() const override { return TraCIAngle { m_cache->get<VAR_ANGLE>() }; }
    double getSpeed() const override { return m_cache->get<VAR_SPEED>(); }

private:
    std::shared_ptr<VehicleCache> m_cache;
};

} // namespace


Define_Module(BasicNodeManager)

const simsignal_t BasicNodeManager::addNodeSignal = cComponent::registerSignal("traci.node.add");
const simsignal_t BasicNodeManager::updateNodeSignal = cComponent::registerSignal("traci.node.update");
const simsignal_t BasicNodeManager::removeNodeSignal = cComponent::registerSignal("traci.node.remove");
const simsignal_t BasicNodeManager::addVehicleSignal = cComponent::registerSignal("traci.vehicle.add");
const simsignal_t BasicNodeManager::updateVehicleSignal = cComponent::registerSignal("traci.vehicle.update");
const simsignal_t BasicNodeManager::removeVehicleSignal = cComponent::registerSignal("traci.vehicle.remove");

void BasicNodeManager::initialize()
{
    Core* core = inet::getModuleFromPar<Core>(par("coreModule"), this);
    subscribeTraCI(core);
    m_api = &core->getLiteAPI();
    m_mapper = inet::getModuleFromPar<ModuleMapper>(par("mapperModule"), this);
    m_nodeIndex = 0;
    m_objectSinkModule = par("objectSinkModule").stringValue();
    m_subscriptions = inet::getModuleFromPar<SubscriptionManager>(par("subscriptionsModule"), this);
}

void BasicNodeManager::finish()
{
    m_api = nullptr;
    unsubscribeTraCI();
    cSimpleModule::finish();
}

void BasicNodeManager::traciInit()
{
    using namespace traci::constants;
    m_boundary = Boundary { m_api->simulation().getNetBoundary() };
    m_subscriptions->subscribeSimulationVariables(sSimulationVariables);
    m_subscriptions->subscribeVehicleVariables(sVehicleVariables);

    // insert already running vehicles
    for (const std::string& id : m_api->vehicle().getIDList()) {
        addVehicle(id);
    }
}

void BasicNodeManager::traciStep()
{
    auto sim_cache = m_subscriptions->getSimulationCache();
    ASSERT(checkTimeSync(*sim_cache, omnetpp::simTime()));

    const auto& departed = sim_cache->get<VAR_DEPARTED_VEHICLES_IDS>();
    EV_DETAIL << "TraCI: " << departed.size() << " vehicles departed" << endl;
    for (const auto& id : departed) {
        addVehicle(id);
    }

    const auto& arrived = sim_cache->get<VAR_ARRIVED_VEHICLES_IDS>();
    EV_DETAIL << "TraCI: " << arrived.size() << " vehicles arrived" << endl;
    for (const auto& id : arrived) {
        removeVehicle(id);
    }

    for (auto& vehicle : m_vehicles) {
        const std::string& id = vehicle.first;
        MovingObjectSink* sink = vehicle.second;
        updateVehicle(id, sink);
    }

    emit(updateNodeSignal, getNumberOfNodes());
}

void BasicNodeManager::traciClose()
{
    for (unsigned i = m_nodes.size(); i > 0; --i) {
        removeNodeModule(m_nodes.begin()->first);
    }
}

void BasicNodeManager::addVehicle(const std::string& id)
{
    NodeInitializer init = [this, &id](cModule* module) {
        MovingObjectSink* vehicle = getVehicleSink(module);
        auto& traci = m_api->vehicle();
        vehicle->initializeSink(m_api, id, m_boundary, m_subscriptions->getVehicleCache(id));
        vehicle->initializeObject(traci.getPosition(id), TraCIAngle { traci.getAngle(id) }, traci.getSpeed(id));
        m_vehicles[id] = vehicle;
    };

    emit(addVehicleSignal, id.c_str());
    cModuleType* type = m_mapper->getMovingObjectType(*this, id);
    if (type != nullptr) {
        addNodeModule(id, type, init);
    } else {
        m_vehicles[id] = nullptr;
    }
}

void BasicNodeManager::removeVehicle(const std::string& id)
{
    emit(removeVehicleSignal, id.c_str());
    removeNodeModule(id);
    m_vehicles.erase(id);
}

void BasicNodeManager::updateVehicle(const std::string& id, MovingObjectSink* sink)
{
    auto vehicle = m_subscriptions->getVehicleCache(id);
    VehicleObjectImpl update(vehicle);
    emit(updateVehicleSignal, id.c_str(), &update);
    if (sink) {
        sink->updateObject(vehicle->get<VAR_POSITION>(),
                TraCIAngle { vehicle->get<VAR_ANGLE>() },
                vehicle->get<VAR_SPEED>());
    }
}

cModule* BasicNodeManager::createModule(const std::string&, cModuleType* type)
{
    cModule* module = type->create("node", getSystemModule(), m_nodeIndex, m_nodeIndex);
    ++m_nodeIndex;
    return module;
}

cModule* BasicNodeManager::addNodeModule(const std::string& id, cModuleType* type, NodeInitializer& init)
{
    cModule* module = createModule(id, type);
    module->finalizeParameters();
    module->buildInside();
    m_nodes[id] = module;
    init(module);
    module->scheduleStart(simTime());
    module->callInitialize();
    emit(addNodeSignal, id.c_str(), module);

    return module;
}

void BasicNodeManager::removeNodeModule(const std::string& id)
{
    cModule* module = getNodeModule(id);
    if (module) {
        emit(removeNodeSignal, id.c_str(), module);
        module->callFinish();
        module->deleteModule();
        m_nodes.erase(id);
    } else {
        EV_DEBUG << "Node with id " << id << " does not exist, no removal\n";
    }
}

cModule* BasicNodeManager::getNodeModule(const std::string& id)
{
    auto found = m_nodes.find(id);
    return found != m_nodes.end() ? found->second : nullptr;
}

std::size_t BasicNodeManager::getNumberOfNodes() const
{
    return m_nodes.size();
}

MovingObjectSink* BasicNodeManager::getVehicleSink(cModule* node)
{
    ASSERT(node);
    cModule* module = node->getModuleByPath(m_objectSinkModule.c_str());
    if (!module) {
        throw cRuntimeError("No module found at %s relative to %s", m_objectSinkModule.c_str(), node->getFullPath().c_str());
    }

    auto* mobility = dynamic_cast<MovingObjectSink*>(module);
    if (!mobility) {
        throw cRuntimeError("Module %s is not a VehicleSink", module->getFullPath().c_str());
    }

    return mobility;
}

MovingObjectSink* BasicNodeManager::getVehicleSink(const std::string& id)
{
    auto found = m_vehicles.find(id);
    return found != m_vehicles.end() ? found->second : nullptr;
}

} // namespace traci
