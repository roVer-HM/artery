#ifndef INETRADIODRIVER_H_PJFDM4JW
#define INETRADIODRIVER_H_PJFDM4JW

#include <artery/nic/RadioDriverBase.h>
#include <omnetpp/clistener.h>
#include <inet/common/Protocol.h>
#include <inet/common/ProtocolGroup.h>
#include <inet/common/ProtocolTag_m.h>

// forward declaration
namespace inet {
namespace ieee80211 {
class Ieee80211Mac;
} // namespace ieee80211
} // namespace inet

namespace artery
{

class InetRadioDriver : public RadioDriverBase, public omnetpp::cListener
{
    public:
        int numInitStages() const override;
        void initialize(int stage) override;
        void handleMessage(omnetpp::cMessage*) override;
        static const inet::Protocol geonet;

    protected:
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, double, omnetpp::cObject*) override;
        void handleDataIndication(omnetpp::cMessage*);
        void handleDataRequest(omnetpp::cMessage*) override;

    private:
        inet::ieee80211::Ieee80211Mac* mLinkLayer = nullptr;
};

} // namespace artery

#endif /* INETRADIODRIVER_H_PJFDM4JW */

