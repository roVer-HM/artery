#ifndef ARTERY_VASERVICE_H
#define ARTERY_VASERVICE_H

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/vam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>

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
    bool checkRedundancyMitigation(const omnetpp::SimTime&) const;
    bool checkRedundancyTimeDelta(const omnetpp::SimTime&) const;
    void sendVam(const omnetpp::SimTime&);
    omnetpp::SimTime genVamDcc();
    vanetza::asn1::Vam generateVam(const MovingNodeDataProvider&, uint16_t genDeltaTime);
    void addLowFrequencyContainer(vanetza::asn1::Vam&);

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
    const StationType* mStationType;
    long mSizeClass;
    long mVruDeviceUsage;
    bool mDccRestriction;
    vanetza::units::Length mMinLatDistance;
    vanetza::units::Length mMinLongDistance;
    vanetza::units::Length mMinVertDistance;
    double mTrajectoryInterceptionThreshold;
    double mLastVamTrajectoryInterception[8];
};
}

#endif /* ARTERY_VASERVICE_H */
