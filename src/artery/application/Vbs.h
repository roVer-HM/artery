#ifndef ARTERY_VBS_H
#define ARTERY_VBS_H

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

class MovingNodeDataProvider;
class NetworkInterfaceTable;

class Vbs: public ItsG5BaseService
{
public:
    Vbs();
    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
    void trigger() override;

private:
    void checkTriggerConditions(const omnetpp::SiTime&);
    bool checkSpeedDelta() const;
    bool checkReferencePositionDelta() const;
    bool checkOrientationDelta() const;
    void sendVam(const omnetpp::SimTime&);
    omnetpp::SimTime genVamDcc();
    vanetza::asn1::Vam generateVam(const MovingNodeDataProvider&, uint16_t genDeltaTime);
    void addLfContainer(vanetza::asn1::Vam&);

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
    vanetza::units::Length mReferencePointThreshold;
    vanetza::units::Angle mLastVamOrientation;
    vanetza::units::Velocity mLastVamSpeed;
    Position mLastVamReferencePosition;
    uint8_t mNumSkipVamRedundancy;
    int mVruProfile;
    long mVruSubProfile;
    int mVruRole;
    int mClusterState;
    StationType mStationType;
    long mSizeClass;
    long mVruDeviceUsage;


    vanetza::units::Length mMinLatDistance;
    vanetza::units::Length mMinLongDistance;
    vanetza::units::length mMinVertDistance;
    uint8_t mTrajectoryInterceptionThreshold;
    uint8_t mLastVamTrajectoryInterception[8];
};
}

#endif /*ARTERY_VBS_H
