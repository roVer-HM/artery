#ifndef ARTERY_MOBILITYBASE_H_1SQMAVHF
#define ARTERY_MOBILITYBASE_H_1SQMAVHF

#include "artery/traci/ControllableVehicle.h"
#include "traci/PersonSink.h"
#include "traci/VehicleSink.h"
#include "traci/VariableCache.h"
#include <omnetpp/clistener.h>
#include "traci/API.h"
#include "artery/utility/Geometry.h"
#include <omnetpp/ccomponent.h>
#include <memory>

namespace artery
{

class MobilityBase :
    public ControllableObject // for controlling the any moving object via TraCI
{
public:
    // traci::ControllableVehicle
    traci::MovingNodeController* getControllerBase() override;

    // generic signal for mobility state changes
    static omnetpp::simsignal_t stateChangedSignal;

protected:
    std::string mObjectId;
    std::unique_ptr<traci::MovingNodeController> mController;

    virtual void initialize(const Position&, Angle, double speed) = 0;
    virtual void update(const Position&, Angle, double speed) = 0;

    std::shared_ptr<traci::API> mTraci;
    traci::Boundary mNetBoundary;
};

} // namespace artery

#endif /* ARTERY_MOBILITYBASE_H_1SQMAVHF */
