#ifndef INETRADIODRIVER_H_PJFDM4JW
#define INETRADIODRIVER_H_PJFDM4JW

#include <artery/nic/RadioDriverBase.h>
#include <omnetpp/clistener.h>
#include <inet/common/Protocol.h>
#include <inet/common/ProtocolGroup.h>
#include <inet/common/ProtocolTag_m.h>

// forward declaration
namespace inet {
namespace ieee80211 { class Ieee80211Mac; }
namespace physicallayer { class Ieee80211Radio; }
} // namespace inet

namespace artery
{

const inet::Protocol* getGeoNetProtocol();

class InetRadioDriver : public RadioDriverBase, public omnetpp::cListener
{
    public:
        int numInitStages() const override;
        using RadioDriverBase::initialize;
        void initialize(int stage) override;
        void handleMessage(omnetpp::cMessage*) override;
//        static const inet::Protocol geonet;
        static const int GEONET_ID;

    protected:
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, double, omnetpp::cObject*) override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, long, omnetpp::cObject*) override;
        void handleDataIndication(omnetpp::cMessage*);
        void handleDataRequest(omnetpp::cMessage*) override;

    protected:
        const inet::Protocol* geonetProtocol;

    private:
        inet::ieee80211::Ieee80211Mac* mLinkLayer = nullptr;
        inet::physicallayer::Ieee80211Radio* mRadio = nullptr;
        int mChannelNumber = 0;
};

} // namespace artery

#endif /* INETRADIODRIVER_H_PJFDM4JW */

