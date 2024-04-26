#ifndef ARTERY_MOBILITYBASE_H_1SQMAVHF
#define ARTERY_MOBILITYBASE_H_1SQMAVHF

#include "traci/API.h"
#include "artery/utility/Geometry.h"
#include "artery/traci/ControllableObject.h"
#include <omnetpp/ccomponent.h>
#include <memory>

namespace artery
{

class MobilityBase : public ControllableObject
{
public:
    // traci::ControllableVehicle
    traci::Controller* getControllerBase() override;

    // generic signal for mobility state changes
    static omnetpp::simsignal_t stateChangedSignal;
    virtual const std::string& getTraciId() const = 0;

protected:
    std::string mObjectId;
    std::unique_ptr<traci::Controller> mController;

    virtual void initialize(const Position&, Angle, double speed) = 0;
    virtual void update(const Position&, Angle, double speed) = 0;

    std::shared_ptr<traci::API> mTraci;
    traci::Boundary mNetBoundary;
};

} // namespace artery

#endif /* ARTERY_MOBILITYBASE_H_1SQMAVHF */
