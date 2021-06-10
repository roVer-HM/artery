#include "artery/traci/MobilityBase.h"
#include "artery/traci/Cast.h"

using namespace traci;

namespace artery
{

omnetpp::simsignal_t MobilityBase::stateChangedSignal = omnetpp::cComponent::registerSignal("mobilityStateChanged");

traci::MovingNodeController* MobilityBase::getControllerBase(){
    ASSERT(mController);
    return mController.get();
}

} // namespace artery
