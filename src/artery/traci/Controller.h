#ifndef CONTROLLER_H_AXBS5NQM
#define CONTROLLER_H_AXBS5NQM

#include "artery/utility/Geometry.h"
#include "traci/API.h"
#include "traci/VariableCache.h"
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/length.hpp>
#include <vanetza/units/velocity.hpp>
#include <string>

namespace traci
{

class VariableCache;

class Controller
{
public:
    using Acceleration = vanetza::units::Acceleration;
    using Length = vanetza::units::Length;
    using Velocity = vanetza::units::Velocity;

    virtual const std::string& getTraciId() const;
    virtual const std::shared_ptr<VariableCache>& getCache() const;
    virtual const traci::Boundary& getBoundary() const;
    virtual std::string getTypeId() const;

    virtual artery::Position getPosition() const;
    virtual artery::GeoPosition getGeoPosition() const;
    virtual artery::Angle getHeading() const;
    virtual Velocity getSpeed() const;
    virtual Velocity getMaxSpeed() const = 0;
    virtual void setMaxSpeed(Velocity) = 0;
    virtual void setSpeed(Velocity) = 0;


    virtual Length getLength() const;
    virtual Length getWidth() const;

    std::shared_ptr<traci::API> getTraCI() { return m_traci; }
    std::shared_ptr<const traci::API> getTraCI() const { return m_traci; }

    virtual ~Controller() {}

protected:
    Controller(std::shared_ptr<traci::API>, std::shared_ptr<VariableCache> cache);

    std::shared_ptr<traci::API> m_traci;
    traci::Boundary m_boundary;
    std::shared_ptr<VariableCache> m_cache;
};

} // namespace traci

#endif /* CONTROLLER_H_AXBS5NQM */

