#include "artery/application/den/SuspendableUseCase.h"
#include "artery/application/DenService.h"
#include "artery/application/VehicleDataProvider.h"

namespace artery
{
namespace den
{

void SuspendableUseCase::initialize(int stage)
{
    UseCase::initialize(stage);
    if (stage == 0) {
        mVdp = &mService->getFacilities().get_const<VehicleDataProvider>();
        setDetectionBlockingInterval(par("detectionBlockingInterval"));
    }
}

const MovingNodeDataProvider* SuspendableUseCase::dataProvider()
{
    return mVdp;
}

void SuspendableUseCase::setDetectionBlockingInterval(omnetpp::SimTime block)
{
    mDetectionBlockingInterval = std::max(omnetpp::SimTime::ZERO, block);
}

bool SuspendableUseCase::isDetectionBlocked()
{
    if (mDetectionBlockingSince) {
        const auto now = omnetpp::simTime();
        if (*mDetectionBlockingSince + mDetectionBlockingInterval < now) {
            mDetectionBlockingSince.reset();
            return false;
        } else {
            return true;
        }
    } else {
        return false;
    }
}

void SuspendableUseCase::blockDetection()
{
    mDetectionBlockingSince = omnetpp::simTime();
}

} // namespace den
} // namespace artery
