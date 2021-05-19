/*
 * MovingNodeDataProvider.h
 *
 *  Created on: Aug 5, 2020
 *      Author: vm-sts
 */

#ifndef ARTERY_APPLICATION_MOVINGNODEDATAPROVIDER_H_
#define ARTERY_APPLICATION_MOVINGNODEDATAPROVIDER_H_

#include "artery/traci/MovingNodeController.h"
#include "artery/application/VehicleKinematics.h"
#include "artery/utility/Geometry.h"
#include <omnetpp/simtime.h>
#include <boost/circular_buffer.hpp>
#include <boost/units/systems/si/angular_acceleration.hpp>
#include <vanetza/geonet/station_type.hpp>
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/units/angular_velocity.hpp>
#include <vanetza/units/curvature.hpp>
#include <cstdint>
#include <map>

namespace artery
{

class MovingNodeDataProvider {
public:
    using StationType = vanetza::geonet::StationType;

    MovingNodeDataProvider();
    MovingNodeDataProvider(uint32_t id);
    virtual ~MovingNodeDataProvider();

    // prevent inadvertent VDP copies
    MovingNodeDataProvider(const MovingNodeDataProvider&) = delete;
    MovingNodeDataProvider& operator=(const MovingNodeDataProvider&) = delete;

    void update(const VehicleKinematics& dynamics);
    omnetpp::SimTime updated() const { return mLastUpdate; }

    uint32_t station_id() const { return mStationId; }
    const Position& position() const { return mVehicleKinematics.position; }
    vanetza::units::GeoAngle longitude() const { return mVehicleKinematics.geo_position.longitude; } // positive for east
    vanetza::units::GeoAngle latitude() const { return mVehicleKinematics.geo_position.latitude; } // positive for north
    vanetza::units::Velocity speed() const { return mVehicleKinematics.speed; }
    vanetza::units::Acceleration acceleration() const { return mVehicleKinematics.acceleration; }
    vanetza::units::Angle heading() const { return mVehicleKinematics.heading; } // degree from north, clockwise
    vanetza::units::AngularVelocity yaw_rate() const { return mVehicleKinematics.yaw_rate; } // left turn positive
    vanetza::units::Curvature curvature() const { return mCurvature; } // 1/m radius, left turn positive
    double curvature_confidence() const { return mConfidence; } // percentage value


    void setStationType(StationType);
    StationType getStationType() const;

private:
    typedef boost::units::quantity<boost::units::si::angular_acceleration> AngularAcceleration;
    void calculateCurvature();
    void calculateCurvatureConfidence();
    double mapOntoConfidence(AngularAcceleration) const;

    uint32_t mStationId;
    StationType mStationType;
    VehicleKinematics mVehicleKinematics;
    vanetza::units::Curvature mCurvature;
    double mConfidence;
    omnetpp::SimTime mLastUpdate;
    boost::circular_buffer<vanetza::units::Curvature> mCurvatureOutput;
    boost::circular_buffer<AngularAcceleration> mCurvatureConfidenceOutput;
    vanetza::units::AngularVelocity mCurvatureConfidenceInput;
    static const std::map<AngularAcceleration, double> mConfidenceTable;
};

} /* namespace artery */

#endif /* ARTERY_APPLICATION_MOVINGNODEDATAPROVIDER_H_ */
