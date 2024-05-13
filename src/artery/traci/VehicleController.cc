#include "artery/traci/VehicleController.h"

namespace si = boost::units::si;

namespace traci
{

VehicleController::VehicleController(std::shared_ptr<traci::API> api, const std::string& id) :
    VehicleController(api, std::make_shared<VehicleCache>(api, id))
{
}

VehicleController::VehicleController(std::shared_ptr<traci::API> api, std::shared_ptr<VehicleCache> cache) :
    Controller(api, cache),
    m_type(api->vehicletype, api->vehicle.getTypeID(cache->getId()))
{
}

const std::string& VehicleController::getVehicleId() const
{
    return getTraciId();
}

const VehicleType& VehicleController::getVehicleType() const
{
    return m_type;
}

const std::string VehicleController::getVehicleClass() const
{
    return m_cache->get<libsumo::VAR_VEHICLECLASS>();
}

auto VehicleController::getMaxSpeed() const -> Velocity
{
    return m_cache->get<libsumo::VAR_MAXSPEED>() * si::meter_per_second;
}

void VehicleController::setMaxSpeed(Velocity v)
{
    m_traci->vehicle.setMaxSpeed(getTraciId(), v / si::meter_per_second);
}

void VehicleController::setSpeed(Velocity v)
{
    m_traci->vehicle.setSpeed(getTraciId(), v / si::meter_per_second);
}

void VehicleController::setSpeedFactor(double f)
{
    m_traci->vehicle.setSpeedFactor(getTraciId(), f);
}

void VehicleController::changeTarget(const std::string& edge)
{
    m_traci->vehicle.changeTarget(getTraciId(), edge);
}

} // namespace traci
