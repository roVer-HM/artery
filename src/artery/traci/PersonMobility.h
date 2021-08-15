#ifndef ARTERY_PERSONMOBILITY_H_QSO2ZBKC
#define ARTERY_PERSONMOBILITY_H_QSO2ZBKC

#include "artery/traci/MobilityBase.h"
#include "artery/utility/Geometry.h"
#include "traci/PersonSink.h"
#include "traci/VariableCache.h"
#include "artery/traci/ControlablePerson.h"
#include <string>

namespace artery
{

class PersonMobility :
    public virtual MobilityBase,
    public traci::PersonSink, // for receiving updates from TraCI
    public ControlablePerson
{
public:
    // traci::PersonSink interface
    virtual void initializeSink(std::shared_ptr<traci::API>, std::shared_ptr<traci::PersonCache>, const traci::Boundary&) override;
    virtual void initializePerson(const traci::TraCIPosition&, traci::TraCIAngle, double speed) override;
    virtual void updatePerson(const traci::TraCIPosition&, traci::TraCIAngle, double speed) override;


    // ControlablePerson
    virtual  traci::PersonController* getPersonController() override;

    const std::string& getPersonId() const { return mPersonId; }
    double getSpeed() const { return mSpeed; }
    Angle getHeading() const { return mHeading; }
    Position getPosition() const { return mPosition; }
    GeoPosition getGeoPosition() const;

protected:
    std::shared_ptr<traci::PersonCache> mCache;
    std::string mPersonId;
    Angle mHeading;
    Position mPosition;
    double mSpeed;
};

} // namespace artery

#endif /* ARTERY_PERSONMOBILITY_H_QSO2ZBKC */
