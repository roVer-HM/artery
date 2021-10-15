#include "artery/application/Vbs.h"
#include "artery/application/VaObject.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MovingNodeDataProvider.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <chrono>


namespace artery
{

using namespace omnetpp;

/**
 * Useful constants and methods taken from CaService.
 */

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

static const simsignal_t scSignalVamReceived = cComponent::registerSignal("VamReceived");
static const simsignal_t scSignalVamSent = cComponent::registerSignal("VamSent");
static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(2000);

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
    boost::units::quantity<U> v { q };
    return std::round(v.value());
}

SpeedValue_t buildSpeedValue(const vanetza::units::Velocity& v)
{
    static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
    static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };

    SpeedValue_t speed = SpeedValue_unavailable;
    if (v >= upper) {
        speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
    } else if (v >= lower) {
        speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
    }
    return speed;
}

Define_Module(Vbs)

Vbs::Vbs() :
    mDeviceDataProvider(nullptr),
    mNetworkInterfaceTable(nullptr),
    mTimer(nullptr),
    mGenVamMin {100, SIMTIME_MS},
    mGenVamMax {5000, SIMTIME_MS},
    mGenVam {mGenVamMin},
    mNumSkipVamRedundancy(5)
{
}

void Vbs::initialize() {
    ItsG5BaseService::initialize();

    // initialize facilities layer entities.
    mDeviceDataProvider = &getFacilities().get_const<MovingNodeDataProvider>();
    mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
    mTimer = &getFacilities().get_const<Timer>();
    mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();
    mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CA);

    // initialize generation interval boundaries
    mGenVamMin = par("minInterval");
    mGenVamMax = par("maxInterval");
    mGenVam = mGenVamMin;
    mDccRestriction = par("withDccRestriction");

    // initialize VAM timestamps so that the first generated VAM includes a low frequency container.
    mLastVamTimestamp = simTime();
    mLastVamLfTimestamp = mLastVamTimestapm - artery::simtime_cast(scLowFrequencyContainerInterval);

    // initialize threshold variables
    mOrientationThreshold = vanetza::units::Angle { par("orientationThreshold").doubleValue() * vanetza::units::degree };
    mSpeedThreshold = par("speedThreshold").doubleValue() * vanetza::units::si::meter_per_second;
    mReferencePointThreshold = par("referencePointThreshold").doubleValue() * vanetza::units::si::meter;
    mTrajectoryInterceptionThreshold = par("trajectoryInterceptionThreshold");
    mNumSkipVamRedundancy = par("numSkipVamRedundancy");
    mMinLatDistance = par("minLatDistance").doubleValue() * vanetza::units::si::meter;
    mMinLongDistance = par("minLongDistance").doubleValue() * vanetza::units::si::meter;
    mMinVertDistance = par("minVertDistance").doubleValue() * vanetza::units::si::meter;

    // initialize VRU profile variables.
    mVruDeviceUsage = par("vruDeviceUsage");
    mClusterState = 1;
    mSizeClass = VruSizeClass_medium;
    mVruProfile = VruProfileAndSubprofile_PR_pedestrian;
    mVruSubProfile = VruSubProfilePedestrian_ordinary_pedestrian;
    mStationType = &getStationType();
}

void Vbs::indicate(const vanetza::btp::DataIndication&,
        std::unique_ptr<vanetza::UpPacket>) {
    Enter_Method("indicate");

    Asn1PacketVisitor<vanetza::asn1::Vam> visitor;
    const vanetza::asn1::Vam* vaam = boost::apply_visitor(visitor, *packet);

    if(vam && vam->validate()){
        VaObject obj = visitor.shared_wrapper;
        emit(scSignalVamReceived, &obj);
        // mLocalDynamicMap->updateAwareness(obj);
    }
}

void Vbs::trigger() {
    Enter_Method("trigger");
    checkTriggerConditions(simTime());
}

void Vbs::checkTriggerConditions(const SimTime& T_now) {
    // variable names according to TS 103 300-2 V2.1.1 (section 6.2).
    SimTime& T_GenVam = mDccRestriction ? genVamDcc() : mGenVam;
    const SimTime& T_GenVamMin = mGenVamMin;
    const SimTime& T_GenVamMax = mGenVamMax;
    const SimTime T_elapsed = T_now - mLastVamTimestamp;

    // Time elapsed since last VAM is greater than the minimum time between two VAMs.
    if(T_elapsed >= T_GenVam) {
        // Check if VAM generation & transmission shall be skipped due to redundancy mitigation.
        if(checkRedundancyMitigation(T_elapsed)){
            return;
        }else if (checkSpeedDelta() || checkReferencePositionDelta() || checkOrientationDelta()) {
            sendVam(T_now);
        }else if (T_elasped >= T_GenVamMax){
            sendVam(T_now);
        }
    }
}

bool Vbs::checkSpeedDelta() const {
    return abs(mLastVamSpeed - mDeviceDataProvider->speed()) > mSpeedThreshold;
}

bool Vbs::checkReferencePositionDelta() const {
    return (distance(mLastVamReferencePosition, mDeviceDataProvider->position()) > mReferencePositionThreshold);
}

bool Vbs::checkOrientationDelta() const {
    return abs(mLastVamHeading - mDeviceDataProvider->heading()) > mOrientationThreshold;
}

bool Vbs::checkRedundancyTimeDelta(const SimTime& T_elapsed) {
    return T_elapsed > (mNumSkipVamRedundancy * mGenVamMax);
}

/**
 *  According to TS 103 300-3 V2.1.1 (section 6.43).
 */
bool Vbs::checkRedundancyMitigation(const SimTime& T_elapsed) const {
    // TODO: Implement second set of conditions!
    return (!checkSpeedDelta() && !checkReferencePositionDelta() && !checkOrientationDelta() && checkRedundancyTimeDelta(T_elapsed));
}

/**
 * Modified Version of the CaServices sendCam method to send a VAM.
 */
void Vbs::sendVam(const omnetpp::SimTime&) {
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
 * Use the NetworkInterfaceTable to retrieve the value for T_GenVam from the VBS management entity according to the.
 * channel usage requirements of the Decentralized Congestion Control (DCC).
 */
SimTime Vbs::genVamDcc() {
    // network interface may not be ready yet during initialization, so look it up at this later point.
    auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
    vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
    if (!trc) {
        throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
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
vanetza::asn1::Vam Vbs::generateVam(const MovingNodeDataProvider& ddp, uint16_t genDeltaTime) {
    vanetza::asn1::Vam message;

    ItsPduHeader_t& header = message->header;
    header.protocolVersion = 1;
    header.messageID = ItsPduHeader__messageID_vam;
    header.stationID = vdp.station_id();

    VruAwareness_t& vam = message->vam;
    vam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
    BasicContainer_t& basic = vam.vamParameters.basicContainer;
    VruHighFrequencyContainer_t*& hfc = vam.vamParameters.vruHighFrequencyContainer;

    basic.stationType = mStationType;
    basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    basic.referencePosition.longitude = round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
    basic.referencePosition.latitude = round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
            SemiAxisLength_unavailable;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
            SemiAxisLength_unavailable;

    hfc->heading.headingValue = round(vdp.heading(), decidegree);
    hfc->heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
    hfc->speed.speedValue = buildSpeedValue(vdp.speed());
    hfc->speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
    const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
    // extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well).
    if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
        hfc->longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
    } else {
        hfc->longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
    }
    hfc->longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
    hfc->vruLanePosition->present = VruLanePosition_PR_NOTHING;
    hfc->environment = VruEnvironment_unavailable;
    hfc->deviceUsage = mDeviceUsage;

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
void Vbs::addLowFrequencyContainer(vanetza::asn1::Vam& message) {
    VruLowFrequencyContainer_t*& lfc = message->vam.vamParameters.vruLowFrequencyContainer;

    lfc->profileAndSubprofile->present = mVruProfile;
    lfc->profileAndSubprofile->choice.pedestrian = mVruSubProfile;

    lfc->sizeClass = mSizeClass;

    std::string error;
    if (!message.validate(error)) {
        throw cRuntimeError("Invalid Low Frequency VAM: %s", error.c_str());
    }
}


}
