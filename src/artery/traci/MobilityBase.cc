#include "artery/traci/MobilityBase.h"

using namespace traci;

namespace artery
{

omnetpp::simsignal_t MobilityBase::stateChangedSignal = omnetpp::cComponent::registerSignal("mobilityStateChanged");

traci::Controller* MobilityBase::getControllerBase(){
    ASSERT(mController);
    return mController.get();
}

} // namespace artery
