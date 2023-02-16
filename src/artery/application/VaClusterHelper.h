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
#include <omnetpp.h>

namespace artery
{
namespace cluster
{

double getVamDistance(const vanetza::asn1::Vam&, const vanetza::asn1::Vam&);

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


struct ClusterFormingParameters {
    int numCreateCluster;
    int maxClusterDistance;
    double maxClusterVelocityDifference;
    double maxCombinedClusterDistance;
    int minClusterSize;
    int maxClusterSize;
    int numClusterVAMRepeat;
};

struct ClusterMembershipParameters {
    omnetpp::simtime_t timeClusterContinuity;
    omnetpp::simtime_t timeClusterBreakupWarning;
    omnetpp::simtime_t timeClusterJoinNotification;
    omnetpp::simtime_t timeClusterJoinSuccess;
    omnetpp::simtime_t timeClusterIdChangeNotification;
    omnetpp::simtime_t timeClusterIdPersist;
    omnetpp::simtime_t timeClusterLeaveNotification;
    omnetpp::simtime_t timeCombinedVruClusterOpportunity;
};

class VamScheduler {
public:
    VamScheduler() {}
    virtual ~VamScheduler() {}

    void scheduleUntil(omnetpp::SimTime until, vanetza::asn1::Vam schedVam) {
        scheduledUntil = until;
        vam = schedVam;
    }

    void scheduleOffset(omnetpp::SimTime offsetFromNow, vanetza::asn1::Vam schedVam) {
        scheduleUntil(omnetpp::simTime() + offsetFromNow, schedVam);
    }

    bool hasScheduledVam() {
        return scheduledUntil > omnetpp::simTime();
    }

    vanetza::asn1::Vam getVam() {
        return vam;
    }

private:
    omnetpp::SimTime scheduledUntil;
    vanetza::asn1::Vam vam;
};

class ClusterManager {
public:
    ClusterManager() {
        clusterId = -1;
        isLeader = false;

        lastMsgFromLeader = omnetpp::simTime();
    }

    void joinCluster(ClusterId_t cid) {
        clusterId = cid;
        isLeader = false;
        containsStations.clear();
        lastMsgFromLeader = omnetpp::simTime();
    }

    void makeCluster(ClusterId_t cid) {
        clusterId = cid;
        isLeader = true;
        containsStations.clear();
    }

   void breakCluster() {
        clusterId = -1;
        isLeader = false;
        containsStations.clear();
    }

   bool isClusterLeader() {
       return clusterId != -1 && isLeader;
   }

   bool isMember() {
      return clusterId != -1 && !isLeader;
  }

    void addStation(
            const vanetza::asn1::Vam& vam,
            const vanetza::asn1::Vam& self,
            const ClusterFormingParameters& parameters
    ) {
        StationID_t sid = vam->header.stationID;

        if (!isLeader) {
            throw omnetpp::cRuntimeError("Only the cluster leader can add a station to the cluster");
        }

        ClusterStation station = {
             .stationId = sid,
             .distance = getVamDistance(vam, self)
        };

        if (station.distance > parameters.maxClusterDistance) {
            return;
        }

        if (std::find_if(
                containsStations.begin(),
                containsStations.end(),
                [&station](const ClusterStation& obj) { return obj.stationId == station.stationId; }
            ) == containsStations.end()
        ) containsStations.push_back(station);

        updateRadius(self);
    }

    void removeStation(const vanetza::asn1::Vam& vam, const vanetza::asn1::Vam& self) {
        StationID_t sid = vam->header.stationID;

        if (!isLeader) {
            throw omnetpp::cRuntimeError("Only the cluster leader can remove a station from the cluster");
        }

        containsStations.erase(std::remove_if(containsStations.begin(), containsStations.end(),
                [&sid](ClusterStation& obj) {
                    return obj.stationId == sid;
                }
        ), containsStations.end());

        updateRadius(self);
    }

    void updateRadius(const vanetza::asn1::Vam& self) {
        double maxDistance = 0;

        for (const auto& station : containsStations) {
            double dist = station.distance;

            if (dist > maxDistance) {
                maxDistance = dist;
            }
        }

        radius = maxDistance;
    }

    void receiveMessageFromLeader(const vanetza::asn1::Vam& msg) {
        lastMsgFromLeader = omnetpp::simTime();
    }

    bool hasLostLeader(const ClusterMembershipParameters& membershipParameters) {
        if (isLeader) {
            throw omnetpp::cRuntimeError("Cluster leader can't lose cluster leader (KEKW)");
        }
        if (clusterId < 0) {
            throw omnetpp::cRuntimeError("Not part of a cluster");
        }

        return (omnetpp::simTime() - lastMsgFromLeader) > membershipParameters.timeClusterContinuity;
    }

    void leaveCluster() {
        if (isLeader) {
            throw omnetpp::cRuntimeError("The leader can't leave a cluster");
        }
        clusterId = -1;
    }

    int getClusterSize() {
        return containsStations.size() + 1;
    }

    ClusterId_t getClusterId() {
        return clusterId;
    }

    double getRadius() {
        return radius;
    }

    void addBreakupContainer(vanetza::asn1::Vam& message, ClusterBreakupReason reason, uint16_t genDeltaTime)
    {
        if (!isLeader) {
            throw omnetpp::cRuntimeError("Only the cluster leader can break up clusters");
        }

        message->vam.vamParameters.vruClusterOperationContainer = vanetza::asn1::allocate<VruClusterOperationContainer_t>();
        message->vam.vamParameters.vruClusterOperationContainer->clusterBreakupInfo = vanetza::asn1::allocate<ClusterBreakupInfo_t>();
        ClusterBreakupInfo_t*& bic = message->vam.vamParameters.vruClusterOperationContainer->clusterBreakupInfo;

        bic->breakupTime = (genDeltaTime & (0xFF << 8)) >> 8;
        if (bic->breakupTime == 0) bic->breakupTime = 1;
        bic->clusterBreakupReason = reason;
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

    void addJoinClusterContainer(vanetza::asn1::Vam& message, uint16_t genDeltaTime)
    {
        if (isLeader) {
            throw omnetpp::cRuntimeError("The leader can't join a cluster");
        }
        if (clusterId < 0) {
            throw omnetpp::cRuntimeError("Not part of a cluster, Can't send join message");
        }

        message->vam.vamParameters.vruClusterOperationContainer = vanetza::asn1::allocate<VruClusterOperationContainer_t>();
        message->vam.vamParameters.vruClusterOperationContainer->clusterJoinInfo = vanetza::asn1::allocate<ClusterJoinInfo_t>();
        ClusterJoinInfo_t*& jic = message->vam.vamParameters.vruClusterOperationContainer->clusterJoinInfo;

        jic->clusterId = clusterId;
        jic->joinTime = (genDeltaTime & (0xFF << 8)) >> 8;

        if (jic->joinTime == 0) jic->joinTime = 1;
    }

    void addLeaveClusterContainer(vanetza::asn1::Vam& message, ClusterLeaveReason reason)
    {
        if (isLeader) {
            throw omnetpp::cRuntimeError("The leader can't leave a cluster");
        }
        if (clusterId < 0) {
            throw omnetpp::cRuntimeError("Not part of a cluster, Can't send leave message");
        }

        message->vam.vamParameters.vruClusterOperationContainer = vanetza::asn1::allocate<VruClusterOperationContainer_t>();
        message->vam.vamParameters.vruClusterOperationContainer->clusterLeaveInfo = vanetza::asn1::allocate<ClusterLeaveInfo_t>();
        ClusterLeaveInfo_t*& lic = message->vam.vamParameters.vruClusterOperationContainer->clusterLeaveInfo;

        lic->clusterId = clusterId;
        lic->clusterLeaveReason = reason;
}

private:
    struct ClusterStation {
      StationID_t stationId;
      double distance;
    };

    ClusterId_t clusterId;
    double radius = 0.0;
    bool isLeader;
    std::vector<ClusterStation> containsStations;
    omnetpp::SimTime lastMsgFromLeader;
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
        // Ignore clusters
        if (vam->vam.vamParameters.vruClusterInformationContainer != nullptr) {
            continue;
        }
        if (getVamDistance(vam, me) <= parameters.maxClusterDistance) {
            vamsInCluster.push_back(vam);
        }
    }

    return vamsInCluster.size() >= parameters.numCreateCluster;
}

bool isInBoundingBox(
        const vanetza::asn1::Vam& me,
        const vanetza::asn1::Vam& cluster
) {
    const ClusterBoundingBoxShape_t bbox = cluster->vam.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape;

    // Circular bounding box
    if (bbox.present == ClusterBoundingBoxShape_PR::ClusterBoundingBoxShape_PR_clusterCircle) {
        return getVamDistance(me, cluster) <= bbox.choice.clusterCircle.radius / 10;
    }

    // Other cases are complicated: ignore
    // TODO: Other cases
    return false;
}

// Conditions to determine whether to join or leave a cluster in normal conditions
// ETSI TS 103 300-3
// p. 27
bool canJoinCluster(
    const vanetza::asn1::Vam& me,
    const vanetza::asn1::Vam& cluster,
    ClusterFormingParameters parameters
) {
    // If the compared information fulfils certain conditions, i.e...

    // ...the cluster has not reached its maximal size (cardinality) maxClusterSize...
    if (cluster->vam.vamParameters.vruClusterInformationContainer->clusterProfiles.size >= parameters.maxClusterSize)
        return false;

    // ...the VRU is within the VRU cluster bounding box...
    // ...or at a certain distance maxClusterDistance away from the VRU cluster leader...
    if (!isInBoundingBox(me, cluster) && getVamDistance(me, cluster) > parameters.maxClusterDistance)
        return false;

    // ...and velocity difference less than maxClusterVelocityDifference of own velocity...
    if (getVeloDifference(getVamVelocity(me), getVamVelocity(cluster)) > parameters.maxClusterVelocityDifference)
        return false;

    // ...the VRU device may join the cluster.
    return true;
}

// Helper function that indicates wheter the the received VAM contains a breakup info container
bool clusterWillBreak(const vanetza::asn1::Vam& vam) {
    if (vam->vam.vamParameters.vruClusterOperationContainer == nullptr) {
        return false;
    }
    return vam->vam.vamParameters.vruClusterOperationContainer->clusterBreakupInfo != nullptr;
}



}
}

#endif  // CROWNET_VAM_VACLUSTERHELPER_H
