#ifndef ARTERY_INETMOBILITY_H_SKZPGILS
#define ARTERY_INETMOBILITY_H_SKZPGILS

#include "artery/traci/MobilityBase.h"
#include <inet/mobility/contract/IMobility.h>
#include <omnetpp/csimplemodule.h>

namespace inet { class CanvasProjection; }

namespace artery
{

class InetMobility : public inet::IMobility, public MobilityBase, public ControllableVehicle , public omnetpp::cSimpleModule
{
public:
    // artery::MobilityBase
    void initializeSink(traci::LiteAPI*, const std::string& id, const traci::Boundary&, std::shared_ptr<traci::VariableCache> cache) override;

    // inet::IMobility interface
    double getMaxSpeed() const override;
    inet::Coord getCurrentPosition() override;
    inet::Coord getCurrentVelocity() override;
    inet::Coord getCurrentAcceleration() override;
    inet::Quaternion getCurrentAngularPosition() override;
    inet::Quaternion getCurrentAngularVelocity() override;
    inet::Quaternion getCurrentAngularAcceleration() override;
    inet::Coord getConstraintAreaMax() const override;
    inet::Coord getConstraintAreaMin() const override;

    //
    traci::MovingNodeController* getControllerBase() override {
        return MobilityBase::getControllerBase();
    }

    // omnetpp::cSimpleModule
    void initialize(int stage) override;
    int numInitStages() const override;

protected:
    void refreshDisplay() const override;

private:
    void initialize(const Position& pos, Angle heading, double speed) override;
    void update(const Position& pos, Angle heading, double speed) override;

    inet::Coord mPosition;
    inet::Coord mSpeed;
    inet::Quaternion mOrientation;
    double mAntennaHeight = 0.0;
    omnetpp::cModule* mVisualRepresentation = nullptr;
    const inet::CanvasProjection* mCanvasProjection = nullptr;
};

} // namespace artery

#endif /* ARTERY_INETMOBILITY_H_SKZPGILS */
