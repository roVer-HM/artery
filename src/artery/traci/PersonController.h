/*
 * PersonController.h
 *
 *  Created on: Jul 7, 2021
 *      Author: vm-sts
 */

#ifndef ARTERY_TRACI_PERSONCONTROLLER_H_
#define ARTERY_TRACI_PERSONCONTROLLER_H_

#include <artery/traci/MovingNodeController.h>
#include "artery/traci/MovingNodeController.h"
#include "artery/utility/Geometry.h"
#include "traci/API.h"
#include "traci/VariableCache.h"


namespace traci {

class PersonController: public MovingNodeController {
public:
    using Acceleration = vanetza::units::Acceleration;
    using Length = vanetza::units::Length;
    using Velocity = vanetza::units::Velocity;

    virtual ~PersonController() = default;
    PersonController(std::shared_ptr<traci::API>, const std::string& id);
    PersonController(std::shared_ptr<traci::API>, std::shared_ptr<PersonCache> cache);

    const std::string& getNodeId() const override;
    std::string getTypeId() const override;
    const std::string getNodeClass() const override;

    artery::Position getPosition() const override;
    artery::GeoPosition getGeoPosition() const override;
    artery::Angle getHeading() const override;
    Velocity getSpeed() const override;
    Velocity getMaxSpeed() const override;
    void setMaxSpeed(Velocity) override;
    void setSpeed(Velocity) override;
    void setSpeedFactor(double);

    Length getLength() const override;
    Length getWidth() const override;

    std::shared_ptr<traci::API> getTraCI() { return m_traci; }
    std::shared_ptr<const traci::API> getTraCI() const { return m_traci; }

private:
    std::shared_ptr<traci::API> m_traci;
    traci::Boundary m_boundary;
    std::shared_ptr<PersonCache> m_cache;
};

} /* namespace traci */

#endif /* ARTERY_TRACI_PERSONCONTROLLER_H_ */
