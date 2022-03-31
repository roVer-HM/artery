#include "artery/application/VaService.h"
#include "artery/application/VaObject.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MovingNodeDataProvider.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/utility/simtime_cast.h"
#include "artery/utility/Calculations.h"
#include "artery/utility/IdentityRegistry.h"
#include "veins/base/utils/Coord.h"
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>
#include <boost/geometry/algorithms/distance.hpp>


namespace artery
{

using namespace omnetpp;

static const simsignal_t scSignalVamReceived = cComponent::registerSignal("VamReceived");
static const simsignal_t scSignalVamSent = cComponent::registerSignal("VamSent");
static const simsignal_t scSignalVamAssemblyTime = cComponent::registerSignal("VamAssemblyTime");
static const simsignal_t scSignalVamRefPosDiff = cComponent::registerSignal("VamRefPosDiff");
static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(2000);
static const auto scVamInterval = std::chrono::milliseconds(100);


Define_Module(VaService);

VaService::VaService() :
    mDeviceDataProvider(nullptr),
    mNetworkInterfaceTable(nullptr),
    mTimer(nullptr),
    mStationType(nullptr),
    mGenVamMin {100, SIMTIME_MS},
    mGenVamMax {5000, SIMTIME_MS},
    mGenVam {mGenVamMin},
    mNumSkipVamRedundancy(5)
{
}

void VaService::initialize()
{
    ItsG5BaseService::initialize();

    // initialize facilities layer entities.
    mDeviceDataProvider = &getFacilities().get_const<MovingNodeDataProvider>();
    mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
    mTimer = &getFacilities().get_const<Timer>();
    mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();
    mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::VRU);

    // initialize generation interval boundaries
    mGenVamMin = par("minInterval");
    mGenVamMax = par("maxInterval");
    mGenVam = mGenVamMin;
    mDccRestriction = par("withDccRestriction");

    // initialize LastVamTimestamp so that a VAM is send immediately after the VRU spawns - TS 103 300-3 v2.1.1 (section 6.4.1)
    mLastVamTimestamp = simTime() - artery::simtime_cast(scVamInterval);

    // initialize low frequency container timestamp to send a low frequency container with the first VAM
    mLastVamLfTimestamp = mLastVamTimestamp - artery::simtime_cast(scLowFrequencyContainerInterval);

    // initialize threshold variables
    mOrientationThreshold = vanetza::units::Angle { par("orientationThreshold").doubleValue() * vanetza::units::degree };
    mSpeedThreshold = par("speedThreshold").doubleValue() * vanetza::units::si::meter_per_second;
    mReferencePositionThreshold = par("referencePositionThreshold").doubleValue() * vanetza::units::si::meter;
    mTrajectoryInterceptionThreshold = par("trajectoryInterceptionThreshold");
    mNumSkipVamRedundancy = par("numSkipVamRedundancy");
    mMinLatDistance = par("minLatDistance").doubleValue() * vanetza::units::si::meter;
    mMinLongDistance = par("minLongDistance").doubleValue() * vanetza::units::si::meter;
    mMinVertDistance = par("minVertDistance").doubleValue() * vanetza::units::si::meter;

    // initialize VRU specific attributes
    mVruDeviceUsage = par("vruDeviceUsage");
    mVruDeviceType = VruDeviceType(int(par("vruDeviceType")));
    mClusterState = ClusterState::VruActiveStandalone;
    mVruRole = VruRole::VruRoleOn;
    mSizeClass = VruSizeClass_medium;
    mVruProfile.present = VruProfileAndSubprofile_PR_pedestrian;
    mVruProfile.choice.pedestrian = VruSubProfilePedestrian_ordinary_pedestrian;
    mStationType = &getStationType();
}

void VaService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
    // VRUs with VruRoleOff shall not receive VAMs - TS 103 300-2 v2.1.1 (section 4.2)
    // VRUs with the Tx device type shall not receive VAMS - TS 103 300-2 v2.1.1 (section 4.1)
    if(mVruRole == VruRole::VruRoleOn && mVruDeviceType != VruDeviceType::VruTx ){
        Enter_Method("indicate");
        Asn1PacketVisitor<vanetza::asn1::Vam> visitor;
        const vanetza::asn1::Vam* vam = boost::apply_visitor(visitor, *packet);

        if(vam && vam->validate()){
            VaObject obj = visitor.shared_wrapper;
            emit(scSignalVamReceived, &obj);
            mLocalDynamicMap->updateAwareness(obj);

            // Calculate deviation of the position of the ITS-S that sent the VAM
            uint32_t stationIdVam = (*vam)->header.stationID;

            cModule* pNode = getParentModule()->getParentModule();
            cModule* i = findModuleByPath("World.identiyRegistry");
            IdentityRegistry* idr = check_and_cast<IdentityRegistry*>(i);
            auto identity = idr->lookup<IdentityRegistry::application>(stationIdVam);

            cModule* node = identity->host;
            cModule* vaMod = node->findModuleByPath(".middleware.VaService");

            if(vaMod) {
                VaService* va = check_and_cast<VaService*>(vaMod);
                const MovingNodeDataProvider* vDDP = va->mDeviceDataProvider;
                // Get longitude and latitude from received VAM
                const auto& bc = (*vam)->vam.vamParameters.basicContainer;
                auto pLat = bc.referencePosition.latitude / 1e7 ;
                auto pLong = bc.referencePosition.longitude/ 1e7;
                ReferencePosition r;
                r.longitude = round(vDDP->longitude(), microdegree) * Longitude_oneMicrodegreeEast;
                r.latitude =  round(vDDP->latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
                auto cLat = vDDP->latitude();
                auto cLong = vDDP->longitude();

                // Calculate the position difference
                auto posDiff = vanetza::facilities::distance(bc.referencePosition, vDDP->latitude(), vDDP->longitude());
                EV_INFO << pNode->getFullName() << " received VAM from " << node->getFullName();
                EV_INFO << " | GeoCoordinates (Lat,Long) from VAM (" << pLat << ", " << pLong << ")";
                EV_INFO << " -- from DDP (" << cLat.value() << ", " << cLong.value() << ") | ";
                EV_INFO << node->getFullName() << " has moved " << posDiff.value() << "m in this time.";

                emit(scSignalVamRefPosDiff, posDiff.value());
            }
        }
    }
}


void VaService::trigger()
{
    // VRUs with VruRoleOff shall not send VAMs - TS 103 300-2 v2.1.1 (section 4.2)
    // VRUs with the Rx device type shall not send VAMS - TS 103 300-2 v2.1.1 (section 4.1)
    // VRUs with the cluster state VruPassive shall not send VAMs - TS 103 300-2 v2.1.1 (section 5.4.2.1)
    if(mVruRole == VruRole::VruRoleOn &&
            mVruDeviceType != VruDeviceType::VruRx &&
            mClusterState != ClusterState::VruPassive){
        Enter_Method("trigger");
        checkTriggerConditions(simTime());
    }
}

void VaService::checkTriggerConditions(const SimTime& T_now)
{
    // Variable names according to TS 103 300-2 V2.1.1 (section 6.2).

    SimTime& T_GenVam = mGenVam;

    // Minimum time elapsed between the start of two consecutive VAMs -> 100ms.
    const SimTime& T_GenVamMin = mGenVamMin;

    // Maximum time elapsed between the start of two consecutive VAMs -> 5000ms.
    const SimTime& T_GenVamMax = mGenVamMax;

    const SimTime T_elapsed = T_now - mLastVamTimestamp;

    // Current valid interval for consecutive VAM generation, either mGenVam or set via DCC if enabled.
    const SimTime T_GenVamDcc = mDccRestriction ? genVamDcc() : mGenVam;

    // Time elapsed since last VAM is greater than the minimum time between two VAMs.
    if(T_elapsed >= T_GenVamDcc) {
        // Check if VAM generation & transmission shall be skipped due to redundancy mitigation.
        if(checkRedundancyMitigation(T_elapsed, T_now)){
            return;
        }else if (checkSpeedDelta() || checkReferencePositionDelta() || checkOrientationDelta()) {
            sendVam(T_now);
        }else if (T_elapsed >= T_GenVamMax){
            sendVam(T_now);
        }
    }
}

bool VaService::checkSpeedDelta() const
{
    return abs(mLastVamSpeed - mDeviceDataProvider->speed()) > mSpeedThreshold;
}

bool VaService::checkReferencePositionDelta() const
{
    return (distance(mLastVamReferencePosition, mDeviceDataProvider->position()) > mReferencePositionThreshold);
}

bool VaService::checkOrientationDelta() const
{
    return abs(mLastVamOrientation - mDeviceDataProvider->heading()) > mOrientationThreshold;
}

bool VaService::checkRedundancyTimeDelta(const SimTime& T_elapsed) const
{
    return T_elapsed > (mNumSkipVamRedundancy * mGenVamMax);
}

/**
 * TODO: This method has not been tested yet but should work in theory!
 */
bool VaService::checkRedundancyMessageSent(const SimTime& T_now) const
{
    LocalDynamicMap::VamPredicate stationIdPresent = [&] (const LocalDynamicMap::Vam& msg) {
        bool result;

        const StationID_t stationId = msg->header.stationID;
        const SimTime genDeltaTime = SimTime(msg->vam.generationDeltaTime, SIMTIME_MS);

        if(stationId == mDeviceDataProvider->getStationId() &&
               (T_now - genDeltaTime) <= mGenVam  ) {
            result = true;
        } else {
            result = false;
        }

        return result;
    };

    return mLocalDynamicMap->count(stationIdPresent) >= 1;
}

/**
 *  According to TS 103 300-3 V2.1.1 (section 6.43).
 *  Multiple checks for skipping the current VAM generation event:
 *      - Time elapsed, position change, speed change and orientation change since the last VAM generation do not exceed a certain threshold.
 *      - The information about the ego-VRU has been reported by another ITS-S within T_GenVam.
 *      - The VRU is member of a cluster.
 *  Currently missing the following checks:
 *      - VRU is in a protected or non-drivable area e.g. buildings.
 */
bool VaService::checkRedundancyMitigation(const SimTime& T_elapsed, const SimTime& T_now) const
{
    return (mClusterState == ClusterState::VruPassive || checkRedundancyMessageSent(T_now) ||
            (!checkSpeedDelta() && !checkReferencePositionDelta() && !checkOrientationDelta() && checkRedundancyTimeDelta(T_elapsed)));
}

/**
 * Modified Version of the CaServices sendCam method to create and send a VAM.
 */
void VaService::sendVam(const omnetpp::SimTime& T_now)
{
    uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mDeviceDataProvider->updated()));
    auto vam = generateVam(*mDeviceDataProvider, genDeltaTimeMod);

    mLastVamReferencePosition = mDeviceDataProvider->position();
    mLastVamSpeed = mDeviceDataProvider->speed();
    mLastVamOrientation = mDeviceDataProvider->heading();
    mLastVamTimestamp = T_now;
    if (T_now - mLastVamLfTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
        addLowFrequencyContainer(vam);
        mLastVamLfTimestamp = T_now;
    }

    using namespace vanetza;
    btp::DataRequestB request;
    request.destination_port = btp::ports::VA;
    request.gn.its_aid = aid::VRU;
    request.gn.transport_type = geonet::TransportType::SHB;
    request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
    request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    VaObject obj(std::move(vam));
    emit(scSignalVamSent, &obj);

    using VamByteBuffer = convertible::byte_buffer_impl<asn1::Vam>;
    std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
    std::unique_ptr<convertible::byte_buffer> buffer { new VamByteBuffer(obj.shared_ptr()) };
    payload->layer(OsiLayer::Application) = std::move(buffer);
    this->request(request, std::move(payload));

}

/**
 * Same Method as used in CaService.
 * Use the NetworkInterfaceTable to retrieve the value for T_GenVam from the VaService management entity according to the.
 * channel usage requirements of the Decentralized Congestion Control (DCC).
 */
SimTime VaService::genVamDcc()
{
    // network interface may not be ready yet during initialization, so look it up at this later point.
    auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
    vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
    if (!trc) {
        throw cRuntimeError("No DCC TRC found for VA's primary channel %i", mPrimaryChannel);
    }

    static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
    vanetza::Clock::duration interval = trc->interval(ca_tx);
    SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
    return std::min(mGenVamMax, std::max(mGenVamMin, dcc));
}

/**
 * Returns a VAM, containing the basic container and the high frequency container.
 * Both containers only contain the required fields and all fields necessary for VRU profile 1.
 */
vanetza::asn1::Vam VaService::generateVam(const MovingNodeDataProvider& ddp, uint16_t genDeltaTime)
{
    vanetza::asn1::Vam message;

    ItsPduHeader_t& header = message->header;
    header.protocolVersion = 1;
    header.messageID = ItsPduHeader__messageID_vam;
    header.stationID = ddp.getStationId();

    VruAwareness_t& vam = message->vam;
    vam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
    BasicContainer_t& basic = vam.vamParameters.basicContainer;
    VruHighFrequencyContainer_t*& hfc = vam.vamParameters.vruHighFrequencyContainer;
    hfc = vanetza::asn1::allocate<VruHighFrequencyContainer_t>();

    basic.stationType = (e_StationType)mStationType->getStationType();
    basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    basic.referencePosition.longitude = round(ddp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
    basic.referencePosition.latitude = round(ddp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
            SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
            SemiAxisLength_unavailable;

    hfc->heading.headingValue = round(ddp.heading(), decidegree);
    hfc->heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
    hfc->speed.speedValue = buildSpeedValue(ddp.speed());
    hfc->speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
    const double lonAccelValue = ddp.acceleration() / vanetza::units::si::meter_per_second_squared;
    // extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well).
    if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
        hfc->longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
    } else {
        hfc->longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
    }
    hfc->longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;

    hfc->environment = vanetza::asn1::allocate<VruEnvironment_t>();
    *(hfc->environment) = VruEnvironment_unavailable;

    hfc->deviceUsage = vanetza::asn1::allocate<VruDeviceUsage_t>();
    *(hfc->deviceUsage) = mVruDeviceUsage;

    std::string error;
    if (!message.validate(error)) {
        throw cRuntimeError("Invalid High Frequency VAM: %s", error.c_str());
    }

    return message;
}

/**
 * Adds the low frequency container to an existing VAM.
 * The low frequency container only contains required fields and all fields necessary for VRU profile 1.
 */
void VaService::addLowFrequencyContainer(vanetza::asn1::Vam& message)
{
    VruLowFrequencyContainer_t*& lfc = message->vam.vamParameters.vruLowFrequencyContainer;
    lfc = vanetza::asn1::allocate<VruLowFrequencyContainer_t>();

    VruProfileAndSubprofile_t*& vruProf = lfc->profileAndSubprofile;
    vruProf = vanetza::asn1::allocate<VruProfileAndSubprofile_t>();
    vruProf->present = mVruProfile.present;
    vruProf->choice.pedestrian = mVruProfile.choice.pedestrian;

    lfc->sizeClass = vanetza::asn1::allocate<VruSizeClass_t>();
    *(lfc->sizeClass) = mSizeClass;

    std::string error;
    if (!message.validate(error)) {
        throw cRuntimeError("Invalid Low Frequency VAM: %s", error.c_str());
    }
}

} // namespace artery
