#ifndef ARTERY_PERSONMOBILITY_H_QSO2ZBKC
#define ARTERY_PERSONMOBILITY_H_QSO2ZBKC

#include "artery/traci/ControllablePerson.h"
#include "artery/traci/MobilityBase.h"
#include "traci/PersonSink.h"
#include "traci/VariableCache.h"
#include <string>

namespace artery
{

class PersonMobility :
    public virtual MobilityBase,
    public traci::PersonSink, // for receiving updates from TraCI
    public ControllablePerson // for controlling the person via TraCI
{
public:
    // traci::PersonSink interface
    virtual void initializeSink(std::shared_ptr<traci::API>, std::shared_ptr<traci::PersonCache>, const traci::Boundary&) override;
    virtual void initializePerson(const traci::TraCIPosition&, traci::TraCIAngle, double speed) override;
    virtual void updatePerson(const traci::TraCIPosition&, traci::TraCIAngle, double speed) override;

    const std::string& getTraciId() const override{ return mObjectId; };

    // ControllablePerson
    virtual traci::PersonController* getPersonController() override;

protected:
    // std::string mPersonId; is now part of MobilityBase (mObjectId)
    std::unique_ptr<traci::PersonController> mController;
};

} // namespace artery

#endif /* ARTERY_PERSONMOBILITY_H_QSO2ZBKC */
