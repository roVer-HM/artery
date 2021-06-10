#ifndef ARTERY_VEINSMOBILITY_H_JFWG67L1
#define ARTERY_VEINSMOBILITY_H_JFWG67L1

#include "artery/traci/VehicleMobility.h"
#include <veins/base/modules/BaseMobility.h>
#include <veins/base/utils/Coord.h>

namespace artery
{

class VeinsMobility : public veins::BaseMobility /* Veins */, public ControllableVehicle, public MobilityBase /* Artery */
//class VeinsMobility : public veins::BaseMobility, public artery::VehicleMobility
{
public:
    void initialize(int stage) override;

private:
    void initialize(const Position&, Angle, double speed) override;
    void update(const Position&, Angle, double speed) override;

    // artery::MobilityBase
    // void initializeSink(traci::LiteAPI*, const std::string& id, const traci::Boundary&, std::shared_ptr<traci::VariableCache> cache) override;



    traci::MovingNodeController* getControllerBase() override {
        return MobilityBase::getControllerBase();
    }

    veins::Coord mPosition;
    veins::Coord mDirection;
    double mSpeed;
};

} // namespace artery

#endif /* ARTERY_VEINSMOBILITY_H_JFWG67L1 */

