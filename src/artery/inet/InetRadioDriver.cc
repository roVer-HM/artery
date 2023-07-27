#include "artery/inet/InetRadioDriver.h"
//#include "artery/inet/VanetRxControl.h"
//#include "artery/inet/VanetTxControl.h"
#include "artery/networking/GeoNetIndication.h"
#include "artery/networking/GeoNetRequest.h"
#include "artery/nic/RadioDriverProperties.h"
#include <inet/common/InitStages.h>
#include <inet/common/ModuleAccess.h>
#include <inet/common/packet/chunk/cPacketChunk.h>
#include <inet/linklayer/common/MacAddressTag_m.h>
#include <inet/linklayer/common/UserPriorityTag_m.h>
#include <inet/linklayer/ieee80211/mac/Ieee80211Mac.h>
#include <mutex>
#include <inet/physicallayer/wireless/ieee80211/packetlevel/Ieee80211Radio.h>

using namespace omnetpp;

namespace artery
{

Register_Class(InetRadioDriver)

namespace {

vanetza::MacAddress convert(const inet::MacAddress& mac)
{
	vanetza::MacAddress result;
	mac.getAddressBytes(result.octets.data());
	return result;
}

inet::MacAddress convert(const vanetza::MacAddress& mac)
{
	inet::MacAddress result;
	result.setAddressBytes(const_cast<uint8_t*>(mac.octets.data()));
	return result;
}

std::once_flag register_protocol_flag;


static const simsignal_t radioChannelChangedSignal = cComponent::registerSignal("radioChannelChanged");
static const simsignal_t channelLoadSignal = cComponent::registerSignal("ChannelLoad");

} // namespace

//const inet::Protocol InetRadioDriver::geonet { "GeoNet", "ETSI ITS-G5 GeoNetworking", inet::Protocol::NetworkLayer };
const int InetRadioDriver::GEONET_ID = 0x8947;

const inet::Protocol* getGeoNetProtocol(){

    const inet::Protocol* protocol = inet::ProtocolGroup::getEthertypeProtocolGroup()->findProtocol(InetRadioDriver::GEONET_ID);
    if (protocol == nullptr){
        protocol = new inet::Protocol("GeoNet", "ETSI ITS-G5 GeoNetworking", inet::Protocol::NetworkLayer);

        inet::ProtocolGroup::getEthertypeProtocolGroup()->addProtocol(
        InetRadioDriver::GEONET_ID, protocol);
    }
    return protocol;
}

int InetRadioDriver::numInitStages() const
{
	return inet::NUM_INIT_STAGES;
}

void InetRadioDriver::initialize(int stage)
{
	if (stage == inet::INITSTAGE_LOCAL) {
		RadioDriverBase::initialize();
		cModule* host = inet::getContainingNode(this);
		mLinkLayer = inet::findModuleFromPar<inet::ieee80211::Ieee80211Mac>(par("macModule"), host);
		mLinkLayer->subscribe(channelLoadSignal, this);
		mRadio = inet::findModuleFromPar<inet::physicallayer::Ieee80211Radio>(par("radioModule"), host);
		mRadio->subscribe(radioChannelChangedSignal, this);

		geonetProtocol = getGeoNetProtocol();

	} else if (stage == inet::INITSTAGE_LINK_LAYER) {

		ASSERT(mChannelNumber > 0);
		auto properties = new RadioDriverProperties();
		properties->LinkLayerAddress = convert(mLinkLayer->getAddress());
		properties->ServingChannel = mChannelNumber;
		indicateProperties(properties);
	}
}

void InetRadioDriver::receiveSignal(cComponent* source, simsignal_t signal, double value, cObject*)
{
	if (signal == channelLoadSignal) {
		emit(RadioDriverBase::ChannelLoadSignal, value);
	}
}

void InetRadioDriver::receiveSignal(cComponent* source, simsignal_t signal, long value, cObject*)
{
	if (signal == radioChannelChangedSignal) {
		mChannelNumber = value;
	}
}

void InetRadioDriver::handleMessage(cMessage* msg)
{
	if (msg->getArrivalGate() == gate("lowerLayerIn")) {
		handleDataIndication(msg);
	} else {
		RadioDriverBase::handleMessage(msg);
	}
}

void InetRadioDriver::handleDataRequest(cMessage* msg)
{
	auto request = check_and_cast<GeoNetRequest*>(msg->removeControlInfo());
	auto packet = new inet::Packet("INET GeoNet", inet::makeShared<inet::cPacketChunk>(check_and_cast<cPacket*>(msg)));

	auto addr_tag = packet->addTag<inet::MacAddressReq>();
	addr_tag->setDestAddress(convert(request->destination_addr));
	addr_tag->setSrcAddress(convert(request->source_addr));

	auto proto_tag = packet->addTagIfAbsent<inet::PacketProtocolTag>();
	proto_tag->setProtocol(geonetProtocol);
	assert(request->ether_type.host() == InetRadioDriver::GEONET_ID);

	auto up_tag = packet->addTag<inet::UserPriorityReq>();

	switch (request->access_category) {
		case vanetza::access::AccessCategory::VO:
			up_tag->setUserPriority(7);
			break;
		case vanetza::access::AccessCategory::VI:
			up_tag->setUserPriority(5);
			break;
		case vanetza::access::AccessCategory::BE:
			up_tag->setUserPriority(3);
			break;
		case vanetza::access::AccessCategory::BK:
			up_tag->setUserPriority(1);
			break;
		default:
			error("mapping to user priority (UP) unknown");
	}
	delete request;

	send(packet, "lowerLayerOut");
}

void InetRadioDriver::handleDataIndication(cMessage* msg)
{
	auto packet = check_and_cast<inet::Packet*>(msg);
	auto chunk = packet->peekData<inet::cPacketChunk>();
	auto gn_packet = chunk->getPacket()->dup();

	auto addr_tag = packet->getTag<inet::MacAddressInd>();
//	auto* info = check_and_cast<VanetRxControl*>(packet->removeControlInfo());
	auto* indication = new GeoNetIndication();
	indication->source = convert(addr_tag->getSrcAddress());
	indication->destination = convert(addr_tag->getDestAddress());
	gn_packet->setControlInfo(indication);
	delete msg;

	indicateData(gn_packet);
}

} // namespace artery
