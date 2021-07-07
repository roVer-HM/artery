/*
 * PersonController.cc
 *
 *  Created on: Jul 7, 2021
 *      Author: vm-sts
 */

#include <artery/traci/PersonController.h>
#include "artery/traci/Cast.h"
#include "traci/VariableCache.h"
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/acceleration.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/velocity.hpp>


namespace si = boost::units::si;

namespace traci
{

PersonController::PersonController(std::shared_ptr<traci::API> api, const std::string& id) :
    PersonController(api, std::make_shared<PersonCache>(api, id))
{
}

PersonController::PersonController(std::shared_ptr<traci::API> api, std::shared_ptr<PersonCache> cache) :
    m_traci(api), m_boundary(api->simulation.getNetBoundary()), m_cache(cache)
{
}

const std::string& PersonController::getNodeId() const {
    return m_cache->getId();
}

std::string PersonController::getTypeId() const
{
    return m_cache->get<libsumo::VAR_TYPE>();
}

const std::string PersonController::getNodeClass() const {
    return m_cache->get<libsumo::VAR_VEHICLECLASS>();
}

artery::Position PersonController::getPosition() const
{
    return traci::position_cast(m_boundary, m_cache->get<libsumo::VAR_POSITION>());
}

auto PersonController::getGeoPosition() const -> artery::GeoPosition
{
    TraCIPosition traci_pos = m_cache->get<libsumo::VAR_POSITION>();

    TraCIGeoPosition traci_geo = m_traci->convertGeo(traci_pos);
    artery::GeoPosition geo;
    geo.latitude = traci_geo.latitude * boost::units::degree::degree;
    geo.longitude = traci_geo.longitude * boost::units::degree::degree;
    return geo;
}

auto PersonController::getHeading() const -> artery::Angle
{
    using namespace traci;
    return angle_cast(TraCIAngle { m_cache->get<libsumo::VAR_ANGLE>() });
}

auto PersonController::getSpeed() const -> Velocity
{
    return m_cache->get<libsumo::VAR_SPEED>() * si::meter_per_second;
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

void PersonController::setSpeedFactor(double f)
{
    m_traci->vehicle.setSpeedFactor(m_cache->getId(), f);
}

auto PersonController::getLength() const -> Length
{
    return m_cache->get<libsumo::VAR_LENGTH>() * si::meter;
}

auto PersonController::getWidth() const -> Length
{
    return m_cache->get<libsumo::VAR_WIDTH>() * si::meter;
}

}
