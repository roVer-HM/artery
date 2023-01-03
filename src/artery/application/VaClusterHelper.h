//
// Created by rupp on 02.01.23.
//

#ifndef CROWNET_VAM_VACLUSTERHELPER_H
#define CROWNET_VAM_VACLUSTERHELPER_H

#include <vanetza/asn1/vam.hpp>
#include <math.h>
#include <algorithm>
#include <vanetza/units/length.hpp>
#include <vector>
#include <vanetza/facilities/cam_functions.hpp>
#include <stdlib.h>

namespace artery
{
namespace cluster
{

class Velocity
{
public:
    Velocity(double vx, double vy) {
        x = vx;
        y = vy;
        valid = true;
    }

    Velocity(double vx, double vy, bool isValid) {
        x = vx;
        y = vy;
        valid = isValid;
    }


    Velocity operator -(const Velocity& other) const {
        return Velocity(x - other.x, y - other.y);
    }

    double dotProduct(const Velocity& other) const {
        return x * other.x + y * other.y;
    }

    double mag() const {
        return sqrt(pow(x, 2.0) + pow(y, 2.0));
    }

    double getAngleBetween(const Velocity& other) const {
        return acos(dotProduct(other)/(mag() * other.mag()));
    }

    bool isValid() const {
        return valid;
    }

    static Velocity invalid() {
        return Velocity(-1.0, -1.0, false);
    }

private:
    double x;
    double y;
    bool valid;
};

class ClusterManager {
public:
    ClusterManager() {
        clusterId = -1;
        isLeader = false;
    }

    void joinCluster(int id) {
        clusterId = id;
        isLeader = false;
    }

    void makeCluster() {
        clusterId = generateClusterId();
        isLeader = true;
    }

    unsigned getClusterId() {
        return clusterId;
    }

    void addClusterContainer(vanetza::asn1::Vam& message)
    {
        if (!isLeader) {
            throw omnetpp::cRuntimeError("Only the cluster leader can send cluster VAMs");
        }

        VruClusterInformationContainer_t*& clusC = message->vam.vamParameters.vruClusterInformationContainer;
        clusC = vanetza::asn1::allocate<VruClusterInformationContainer_t>();
        clusC->clusterId = clusterId;
        clusC->clusterBoundingBoxShape.present = ClusterBoundingBoxShape_PR::ClusterBoundingBoxShape_PR_clusterCircle;
        clusC->clusterBoundingBoxShape.choice.clusterCircle.radius = radius * 10;
        clusC->clusterCardinalitySize = containsStations.size() + 1;
    }

private:

    ClusterId_t generateClusterId()
    {
        return static_cast<ClusterId_t>(rand());
    }

    ClusterId_t clusterId;
    int radius = 0;
    bool isLeader;
    std::vector<StationID_t> containsStations;
};


struct ClusterFormingParameters {
    int numCreateCluster;
    int maxClusterDistance;
    double maxClusterVelocityDifference;
    double maxCombinedClusterDistance;
    int minClusterSize;
    int maxClusterSize;
    int numClusterVAMRepeat;
};

const ClusterFormingParameters defaultFormingParameters = {
    .numCreateCluster = 3,
    .maxClusterDistance = 3,
    .maxClusterVelocityDifference = 0.05,
    .maxCombinedClusterDistance = 1.5,
    .minClusterSize = 1,
    .maxClusterSize = 20,
    .numClusterVAMRepeat = 3
};


inline double _deg2rad(double angle)
{
    return (angle * M_PI) / 180.0;
}

Velocity getVamVelocity(const vanetza::asn1::Vam& message)
{
    auto hfc = message->vam.vamParameters.vruHighFrequencyContainer;

    if (hfc == nullptr) {
        return Velocity::invalid();
    }

    long speed = hfc->speed.speedValue;
    long heading = hfc->heading.headingValue;

    // Convert 0.1 deg to radians
    double headingRad = _deg2rad(heading / 10.0);

    Velocity vamVelocity{
        sin(headingRad) * speed / 100.0,
        cos(headingRad) * speed / 100.0
    };

    return vamVelocity;
}

double getVeloDifference(const Velocity v1, const Velocity v2)
{
    if (!v1.isValid() || !v2.isValid()) {
        return -1;
    }
    double speedDifference = 1 - std::min(v1.mag(), v2.mag()) / std::max(v1.mag(), v2.mag());
    double angleDifference = v1.getAngleBetween(v2) / 180;

    return std::max(speedDifference, angleDifference);

}

double getVamDistance(const vanetza::asn1::Vam& vam1, const vanetza::asn1::Vam& vam2)
{
    ReferencePosition pos1 = vam1->vam.vamParameters.basicContainer.referencePosition;
    ReferencePosition pos2 = vam2->vam.vamParameters.basicContainer.referencePosition;

    vanetza::units::Length dist = vanetza::facilities::distance(pos1, pos2);

    return dist / boost::units::si::meter;
}

bool canFormCluster(
    const vanetza::asn1::Vam& me,
    const std::vector<vanetza::asn1::Vam> vams,
    ClusterFormingParameters parameters
)
{
    std::vector<vanetza::asn1::Vam> vamsInCluster;
    // ETSI TS 103 300-3
    // p. 27
    // It is receiving VAMs from numCreateCluster different VRUs not further away than maxClusterDistance.
    for (auto const& vam : vams)
    {
        if (getVamDistance(vam, me) <= parameters.maxClusterDistance)
            vamsInCluster.push_back(vam);
    }

    return vamsInCluster.size() >= parameters.numCreateCluster;
}

bool canJoinCluster(
    const vanetza::asn1::Vam& me,
    const vanetza::asn1::Vam& cluster,
    ClusterFormingParameters parameters
) {
    // TODO: Check max cluster size
    // TODO: Check bounding box
    // If the compared information fulfils certain conditions, i.e. the cluster has not reached its maximal size (cardinality)
    // maxClusterSize, the VRU is within the VRU cluster bounding box or at a certain distance maxClusterDistance away from the
    // VRU cluster leader and velocity difference less than maxClusterVelocityDifference of own velocity, the VRU device may join the cluster.
    if (getVamDistance(me, cluster) > parameters.maxClusterDistance)
        return false;

    if (getVeloDifference(getVamVelocity(me), getVamVelocity(cluster)) > parameters.maxClusterVelocityDifference)
        return false;

    return true;
}



}
}

#endif  // CROWNET_VAM_VACLUSTERHELPER_H
