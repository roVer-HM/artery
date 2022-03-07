#ifndef ARTERY_DEN_COLLISIONRISKSERVICE_H_SCOKLPLY
#define ARTERY_DEN_COLLISIONRISKSERVICE_H_SCOKLPLY

#include "artery/application/den/UseCase.h"
#include "artery/application/LocalDynamicMap.h"
#include "artery/traci/VehicleController.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/units/area.hpp>
#include <vanetza/units/velocity.hpp>
#include <boost/geometry.hpp>

namespace artery
{
namespace den
{

class CollisionRiskService : public UseCase
{
public:
    void initialize(int) override;

    void check() override;
    void indicate(const artery::DenmObject&) override;
    void handleStoryboardTrigger(const StoryboardSignal&) override {};

    vanetza::asn1::Denm createMessage(RequestResponseIndication_t);
    vanetza::btp::DataRequestB createRequest();

protected:
    virtual const MovingNodeDataProvider* dataProvider() override;

    void handleMessage(omnetpp::cMessage* msg) override;

    const MovingNodeDataProvider* mVdp = nullptr;

private:
    bool mPendingRequest = false;
    const LocalDynamicMap* mLocalDynamicMap;
    traci::VehicleController* mVehicleController = nullptr;
    omnetpp::cMessage* mDenValidityExpiredMessage = nullptr;
    vanetza::units::Velocity mLastSpeed;

    void checkPedestrianCollisionLDM();
    vanetza::geonet::GeodeticPosition vectorToLatLon(boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>&);
    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> pointToVector(const double, const double);
    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> pointToGreatCircle(const double, const double, const double);
    void transmitMessage(RequestResponseIndication_t);
    void brake();
};

} // namespace den
} // namespace artery

#endif /* ARTERY_DEN_COLLISIONRISKSERVICE_H_SCOKLPLY */
