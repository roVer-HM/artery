#ifndef ARTERY_DEN_COLLISIONRISKSERVICE_H_SCOKLPLY
#define ARTERY_DEN_COLLISIONRISKSERVICE_H_SCOKLPLY

#include "artery/application/den/UseCase.h"
#include "artery/application/LocalDynamicMap.h"
#include <vanetza/btp/data_request.hpp>

namespace artery
{
namespace den
{

class CollisionRiskService : public UseCase
{
public:
    void initialize(int) override;

    void check() override;
    void indicate(const artery::DenmObject&) override;
    void handleStoryboardTrigger(const StoryboardSignal&) override {};

    vanetza::asn1::Denm createMessage(RequestResponseIndication_t);
    vanetza::btp::DataRequestB createRequest();

protected:
    virtual const MovingNodeDataProvider* dataProvider() override;

    const MovingNodeDataProvider* mVdp = nullptr;

private:
    bool mPendingRequest = false;
    const LocalDynamicMap* mLocalDynamicMap;

    void transmitMessage(RequestResponseIndication_t);
    void brake();
    int calcTTC(vanetza::asn1::Denm&);
};

} // namespace den
} // namespace artery

#endif /* ARTERY_DEN_COLLISIONRISKSERVICE_H_SCOKLPLY */
