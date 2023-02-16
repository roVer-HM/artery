#ifndef ARTERY_VASERVICE_H
#define ARTERY_VASERVICE_H

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include "artery/application/VaClusterHelper.h"
#include <vanetza/asn1/vam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include <stdint.h>
#include <boost/circular_buffer.hpp>
#include <memory>

namespace artery
{

/**
 * Enum for VRU cluster states according to TS 103 300-2 v2.1.1 (section 5.4.2.1)
 */
enum class ClusterState
{
    VruIdle,
    VruActiveStandalone,
    VruActiveClusterLeader,
    VruPassive
};

/**
 * Enum for VRU role according to TS 103 300-2 v2.1.1 (section 4.2)
 */
enum class VruRole
{
    VruRoleOff,
    VruRoleOn
};

/**
 * Enum for VRU device type specification according to TS 103 300-2 v2.1.1 (section 4.1)
 */
enum class VruDeviceType
{
    VruRx,
    VruTx,
    VruSt
};

class MovingNodeDataProvider;
class NetworkInterfaceTable;

class VaService: public ItsG5BaseService
{
public:
    VaService();
    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
    void trigger() override;

private:
    void checkTriggerConditions(const omnetpp::SimTime&);
    bool checkSpeedDelta() const;
    bool checkReferencePositionDelta() const;
    bool checkOrientationDelta() const;
    bool checkRedundancyMitigation(const omnetpp::SimTime&, const omnetpp::SimTime&) const;
    bool checkRedundancyMessageSent(const omnetpp::SimTime&) const;
    bool checkRedundancyTimeDelta(const omnetpp::SimTime&) const;
    void sendVam(const omnetpp::SimTime&);
    omnetpp::SimTime genVamDcc();
    vanetza::asn1::Vam generateVam(const MovingNodeDataProvider&, uint16_t genDeltaTime);
    void addLowFrequencyContainer(vanetza::asn1::Vam&);
    void addHeader(vanetza::asn1::Vam&, const MovingNodeDataProvider&);
    void addBasicContainer(vanetza::asn1::Vam&, const MovingNodeDataProvider&);
    uint16_t generationTime();
    void handleClustering(const vanetza::asn1::Vam*);
    vanetza::asn1::Vam getJoinClusterVam();
    vanetza::asn1::Vam getLeaveClusterVam(ClusterLeaveReason);
    vanetza::asn1::Vam getBreakupClusterVam(ClusterBreakupReason);
    void sendVamRequest(vanetza::asn1::Vam&);

    // void scheduleVam(const vanetza::asn1::Vam& vam, omnetpp::SimTime timeOffset);


    const MovingNodeDataProvider* mDeviceDataProvider;
    const Timer* mTimer;
    artery::LocalDynamicMap* mLocalDynamicMap;
    ChannelNumber mPrimaryChannel = channel::CCH;
    const NetworkInterfaceTable* mNetworkInterfaceTable;
    omnetpp::SimTime mGenVam;
    omnetpp::SimTime mGenVamMin;
    omnetpp::SimTime mGenVamMax;
    omnetpp::SimTime mLastVamTimestamp;
    omnetpp::SimTime mLastVamLfTimestamp;
    vanetza::units::Angle mOrientationThreshold;
    vanetza::units::Velocity mSpeedThreshold;
    vanetza::units::Length mReferencePositionThreshold;
    vanetza::units::Angle mLastVamOrientation;
    vanetza::units::Velocity mLastVamSpeed;
    Position mLastVamReferencePosition;
    uint8_t mNumSkipVamRedundancy;
    VruProfileAndSubprofile_t mVruProfile;
    VruRole mVruRole;
    ClusterState mClusterState;
    VruDeviceType mVruDeviceType;
    const StationType* mStationType;
    long mSizeClass;
    long mVruDeviceUsage;
    bool mDccRestriction;
    vanetza::units::Length mMinLatDistance;
    vanetza::units::Length mMinLongDistance;
    vanetza::units::Length mMinVertDistance;
    double mTrajectoryInterceptionThreshold;
    double mLastVamTrajectoryInterception[8];

    std::unique_ptr<cluster::ClusterManager> mClusterManager;
    cluster::ClusterFormingParameters clusterParameters;
    cluster::ClusterMembershipParameters membershipParameters;

    bool mCanLeadCluster;
    bool mSendsVams;
    boost::circular_buffer<double> mHeadingAvg;

    cluster::VamScheduler vamScheduler;

    omnetpp::cOutVector vVamX;
    omnetpp::cOutVector vVamY;
    omnetpp::cOutVector vVamId;
    omnetpp::cOutVector vVamCluster;
    omnetpp::cOutVector vVamH;
    omnetpp::cOutVector vVamBBoxSize;
    omnetpp::cOutVector vVamType;

    omnetpp::cOutVector vSelfX;
    omnetpp::cOutVector vSelfY;
    omnetpp::cOutVector vSelfCluster;


};
}

#endif /* ARTERY_VASERVICE_H */
