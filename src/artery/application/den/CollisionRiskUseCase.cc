#include "artery/application/den/CollisionRiskUseCase.h"
#include "artery/application/DenService.h"
#include "artery/application/DenmObject.h"
#include "artery/application/MovingNodeDataProvider.h"
#include <vanetza/facilities/cam_functions.hpp>
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/time.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <boost/geometry/arithmetic/cross_product.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>
#include <math.h>
#include <omnetpp/simtime.h>

namespace artery
{
namespace den
{

Define_Module(CollisionRiskService)

constexpr auto microdegree = vanetza::units::degree * boost::units::si::micro;
constexpr double toRadians = boost::math::double_constants::pi / 180;
constexpr long collisionRiskDenValidity = 2;


void CollisionRiskService::initialize(int stage)
{
    UseCase::initialize(stage);
    if (stage == 0) {
        mDenValidityExpiredMessage = new omnetpp::cMessage("DEN validity expired!");
        mVdp = &mService->getFacilities().get_const<MovingNodeDataProvider>();
        mLocalDynamicMap = &mService->getFacilities().get_const<LocalDynamicMap>();
        mVehicleController = &mService->getFacilities().get_mutable<traci::VehicleController>();
    }
}

// Called periodically by DENService through the middleware
void CollisionRiskService::check()
{
    checkPedestrianCollisionLDM();
}

vanetza::geonet::GeodeticPosition CollisionRiskService::vectorToLatLon(boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>& v)
{
    const double latitude = atan2(v.get<2>(), sqrt(v.get<0>() * v.get<0>() + v.get<1>() * v.get<1>()));
    const double longitude = atan2(v.get<1>(), v.get<0>());

    vanetza::units::GeoAngle lat2Deg = latitude / toRadians * vanetza::units::degree;
    vanetza::units::GeoAngle long2Deg = longitude / toRadians * vanetza::units::degree;

    vanetza::geonet::GeodeticPosition pos {
        lat2Deg,
        long2Deg
    };

    return pos;
}

boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> CollisionRiskService::pointToVector(
        const double latitude, const double longitude) {

    const double x = cos(latitude) * cos(longitude);
    const double y = cos(latitude) * sin(longitude);
    const double z = sin(latitude);

    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> vector(x, y, z);
    return vector;
}

boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> CollisionRiskService::pointToGreatCircle(
        const double latitude, const double longitude, const double bearing) {

    const double x = sin(longitude) * cos(bearing) - sin(latitude) * cos(longitude) * sin(bearing);
    const double y = -cos(longitude) * cos(bearing) - sin(latitude) * sin(longitude) * sin(bearing);
    const double z = cos(latitude) * sin(bearing);

    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> greatCircle(x, y, z);
    return greatCircle;
}

// Check for collision risk with VRUs by using the LDM
void CollisionRiskService::checkPedestrianCollisionLDM()
{
    int ttc = -1;
    LocalDynamicMap::VamPredicate collisionRiskWarning = [&] (const LocalDynamicMap::Vam& msg) {
        // TTC threshold in s
        const double epsilon = 0.1;
        const vanetza::units::Length distLimit { 100.0 * vanetza::units::si::meter };
        // time to collision threshold for issuing a warning/DENM, see: ETSI TS 101 539-1 4, Figure 4.1
        const int ttcWarning = 5;
        // time to collision threshold for initiating a collision avoidance action, see: ETSI TS 101 539-1 4, Figure 4.1
        const int ttcAction = 3;

        const auto& bc = msg->vam.vamParameters.basicContainer;
        const auto& hfc = msg->vam.vamParameters.vruHighFrequencyContainer;
        const auto& vdp = *mVdp;
        const auto distToVru = vanetza::facilities::distance(bc.referencePosition, vdp.latitude(), vdp.longitude());

        // Check if VRU is in range for a possible collision
        // Calculations based on reference from  https://www.ffi.no/en/research/n-vector/ - Example 9 -> http://www.movable-type.co.uk/scripts/latlong.html
        if(distToVru <= distLimit) {
            const auto vruVelocity = hfc->speed.speedValue / 100.0;
            const double vruLat = (bc.referencePosition.latitude / 1e7) * toRadians;
            const double vruLong = (bc.referencePosition.longitude / 1e7) * toRadians;
            const double vruHeading = (hfc->heading.headingValue / 10.0) * toRadians;

            // Vector of the VRU based on lat and long
            boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> vVru = pointToVector(vruLat, vruLong);

            // Normalized great circle vector of the VRU based on lat, long and bearing
            const boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> gcVru = pointToGreatCircle(vruLat, vruLong, vruHeading);

            const double vehicleHeading = vdp.heading().value() * toRadians;
            const double vehicleLat = vdp.latitude().value() * toRadians;
            const double vehicleLong = vdp.longitude().value() * toRadians;

            // Vector of the vehicle based on lat and long
            const boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> vVehicle = pointToVector(vehicleLat, vehicleLong);

            // Normalized great circle vector of the VRU based on lat, long and bearing
            const boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> gcVehicle = pointToGreatCircle(vehicleLat, vehicleLong, vehicleHeading);

            // Possible intersections of the vehicle and the VRU
            const boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> i1 = boost::geometry::cross_product(gcVehicle, gcVru);
            const boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> i2 = boost::geometry::cross_product(gcVru, gcVehicle);

            const boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> zeroPoint(0.0, 0.0, 0.0);

            // If the great circles are identical, use a simple ttc calculation
            if(boost::geometry::equals(i1, zeroPoint)) {
                ttc = vdp.speed().value() != vruVelocity ? distToVru.value() / (vdp.speed().value() - vruVelocity) : -1;
            } else {
                // Dot product of the VRUs and vehicles bearings with intersection point one to see if they are pointing in the same direction
                const int s1 = boost::math::sign(boost::geometry::dot_product(boost::geometry::cross_product(gcVru, vVru), i1));
                const int s2 = boost::math::sign(boost::geometry::dot_product(boost::geometry::cross_product(gcVehicle, vVehicle), i1));

                boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> intersection;

                // Determine which intersection point should be taken
                switch(s1 + s2) {
                    case 2: // VRU & vehicle both point towards intersection point 1
                        intersection = i1;
                        break;
                    case -2: // VRU & vehicle both point towards intersection point 2
                        intersection = i2;
                        break;
                    case 0: // VRU & vehicle both point in opposite directions
                        boost::geometry::add_point(vVru, vVehicle);
                        intersection = boost::geometry::dot_product(vVru, i1) > 0 ? i2 : i1;
                        break;
                }

                // Intersection lat and lon in degrees
                vanetza::geonet::GeodeticPosition intersectionInDeg = vectorToLatLon(intersection);
                vanetza::geonet::GeodeticPosition vehicleGeoPos {
                    vdp.latitude(),
                    vdp.longitude()
                };

                const auto distVruIntersect = vanetza::facilities::distance(bc.referencePosition, intersectionInDeg.latitude, intersectionInDeg.longitude);
                const auto distVehicleIntersect = vanetza::geonet::distance(vehicleGeoPos, intersectionInDeg);

                // Calculate TTC
                const double ttcVru = distVruIntersect.value() / vruVelocity;
                const double ttcVehicle = distVehicleIntersect.value() / vdp.speed().value();

                if(abs(ttcVru - ttcVehicle) <= epsilon) {
                    ttc = ttcVru;
                }
            }
        }
        // TTC of less than minTtc is a potential collision risk, see: ETSI EN 103 300-2, 6.5.7
        return ttc >= 0 && ttc <= ttcWarning;
    };

    if(mLocalDynamicMap->count(collisionRiskWarning) >= 1){
        // Send DENM
        transmitMessage(RequestResponseIndication_request);
        mLastSpeed = mVehicleController->getSpeed();
        // Brake to avoid a potential collision
        brake();
        // Self message to trigger reset of vehicle speed
        scheduleAt(omnetpp::simTime() + collisionRiskDenValidity + ttc, mDenValidityExpiredMessage);
    }
}

void CollisionRiskService::handleMessage(omnetpp::cMessage* msg) {
    if (msg == mDenValidityExpiredMessage) {
        mVehicleController->setSpeed(mLastSpeed);
    }
}

void CollisionRiskService::indicate(const artery::DenmObject& denm)
{
    if (denm & CauseCode::CollisionRisk) {
       const vanetza::asn1::Denm& asn1 = denm.asn1();
       if (asn1->denm.alacarte && asn1->denm.alacarte->impactReduction) {
           auto& indication = asn1->denm.alacarte->impactReduction->requestResponseIndication;
           if (indication == RequestResponseIndication_request) {
               transmitMessage(RequestResponseIndication_response);
           }
       }
   }
}

const MovingNodeDataProvider* CollisionRiskService::dataProvider()
{
    return mVdp;
}

void CollisionRiskService::transmitMessage(RequestResponseIndication_t ind)
{
    auto denm = createMessage(ind);
    auto request = createRequest();
    mService->sendDenm(std::move(denm), request);
}

void CollisionRiskService::brake()
{
    const vanetza::units::Velocity brakeSpeed = 0 *  vanetza::units::si::meter_per_second;
    mVehicleController->setSpeed(brakeSpeed);
}

vanetza::asn1::Denm CollisionRiskService::createMessage(RequestResponseIndication_t ind)
{
    auto msg = createMessageSkeleton();
    msg->denm.management.relevanceDistance = vanetza::asn1::allocate<RelevanceDistance_t>();
    *msg->denm.management.relevanceDistance = RelevanceDistance_lessThan100m;
    msg->denm.management.relevanceTrafficDirection = vanetza::asn1::allocate<RelevanceTrafficDirection_t>();
    *msg->denm.management.relevanceTrafficDirection = RelevanceTrafficDirection_allTrafficDirections;
    msg->denm.management.validityDuration = vanetza::asn1::allocate<ValidityDuration_t>();
    *msg->denm.management.validityDuration = collisionRiskDenValidity;
    msg->denm.management.stationType = StationType_unknown; // TODO retrieve type from SUMO

    msg->denm.situation = vanetza::asn1::allocate<SituationContainer_t>();
    msg->denm.situation->informationQuality = 1;
    msg->denm.situation->eventType.causeCode = CauseCodeType_collisionRisk;
    msg->denm.situation->eventType.subCauseCode = 0; // Should be 4 for VRU case

    msg->denm.alacarte = vanetza::asn1::allocate<AlacarteContainer_t>();
    msg->denm.alacarte->impactReduction = vanetza::asn1::allocate<ImpactReductionContainer_t>();
    ImpactReductionContainer_t& irc = *msg->denm.alacarte->impactReduction;
    // TODO approximate values from SUMO vehicle type
    irc.heightLonCarrLeft = HeightLonCarr_unavailable;
    irc.heightLonCarrRight = HeightLonCarr_unavailable;
    irc.posLonCarrLeft = PosLonCarr_unavailable;
    irc.posLonCarrRight = PosLonCarr_unavailable;
    PosPillar_t* pillar = vanetza::asn1::allocate<PosPillar_t>();
    *pillar = PosPillar_unavailable;
    ASN_SEQUENCE_ADD(&irc.positionOfPillars, pillar);
    irc.posCentMass = PosCentMass_unavailable;
    irc.wheelBaseVehicle = WheelBaseVehicle_unavailable;
    irc.turningRadius = TurningRadius_unavailable;
    irc.positionOfOccupants.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(3));
    irc.positionOfOccupants.size = 3;
    irc.positionOfOccupants.bits_unused = 4;
    irc.positionOfOccupants.buf[0] |= 1 << (7 - PositionOfOccupants_row1LeftOccupied);
    irc.positionOfOccupants.buf[1] |= 1 << (15 - PositionOfOccupants_row2NotDetectable);
    irc.positionOfOccupants.buf[1] |= 1 << (15 - PositionOfOccupants_row3NotPresent);
    irc.positionOfOccupants.buf[2] |= 1 << (23 - PositionOfOccupants_row4NotPresent);
    irc.posFrontAx = PosFrontAx_unavailable;
    irc.vehicleMass = VehicleMass_unavailable;
    irc.requestResponseIndication = ind;

    return msg;
}

vanetza::btp::DataRequestB CollisionRiskService::createRequest()
{
    namespace geonet = vanetza::geonet;
    using vanetza::units::si::seconds;
    using vanetza::units::si::meter;

    vanetza::btp::DataRequestB request;
    request.gn.traffic_class.tc_id(0);
    request.gn.maximum_hop_limit = 1;

    geonet::DataRequest::Repetition repetition;
    repetition.interval = 0.1 * seconds;
    repetition.maximum = 0.3 * seconds;
    request.gn.repetition = repetition;

    geonet::Area destination;
    geonet::Circle destination_shape;
    destination_shape.r = 100.0 * meter;
    destination.shape = destination_shape;
    destination.position.latitude = mVdp->latitude();
    destination.position.longitude = mVdp->longitude();
    request.gn.destination = destination;

    return request;
}


} // namespace den
} // namespace artery
