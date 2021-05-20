//
// Copyright (C) 2014 Raphael Riebl <raphael.riebl@thi.de>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef ARTERY_VEHICLEDATAPROVIDER_H_
#define ARTERY_VEHICLEDATAPROVIDER_H_

#include "artery/application/MovingNodeDataProvider.h"
#include "artery/traci/VehicleController.h"


namespace artery
{

class VehicleDataProvider: public MovingNodeDataProvider
{

public:
    VehicleDataProvider();
    VehicleDataProvider(uint32_t id);
/*
	public:
		using StationType = vanetza::geonet::StationType;

		VehicleDataProvider(uint32_t id);

		// prevent inadvertent VDP copies
		VehicleDataProvider(const VehicleDataProvider&) = delete;
		VehicleDataProvider& operator=(const VehicleDataProvider&) = delete;

		void update(const VehicleKinematics&);
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
>>>>>>> 71afcd21624c77b863cd35453bec88f00a168033*/

};

} // namespace artery

#endif /* ARTERY_VEHICLEDATAPROVIDER_H_ */
