#include "artery/traci/PersonController.h"

namespace si = boost::units::si;

namespace traci
{

PersonController::PersonController(std::shared_ptr<traci::API> api, const std::string& id) :
    PersonController(api, std::make_shared<PersonCache>(api, id))
{
}

PersonController::PersonController(std::shared_ptr<traci::API> api, std::shared_ptr<PersonCache> cache) :
    Controller(api, cache)
{
}

const std::string& PersonController::getPersonId() const
{
    return getTraciId();
}

auto PersonController::getMaxSpeed() const -> Velocity
{
    return m_cache->get<libsumo::VAR_MAXSPEED>() * si::meter_per_second;
}

void PersonController::setMaxSpeed(Velocity v)
{
    m_traci->vehicle.setMaxSpeed(m_cache->getId(), v / si::meter_per_second);
}

void PersonController::setSpeed(Velocity v)
{
    m_traci->vehicle.setSpeed(m_cache->getId(), v / si::meter_per_second);
}


const std::string PersonController::getVehicle() const
{
    return m_cache->get<libsumo::VAR_VEHICLE>();
}

const bool PersonController::isDriving() const
{
    return getVehicle().empty() ? false : true;
}

} // namespace traci
