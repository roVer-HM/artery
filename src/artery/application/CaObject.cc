#include <artery/application/CaObject.h>

namespace artery
{

using namespace vanetza::asn1;

Register_Abstract_Class(CaObject)
using namespace omnetpp;

class CamStationIdResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cam = dynamic_cast<CaObject*>(object)) {
            const auto id = cam->asn1()->header.stationID;
            fire(this, t, id, details);
        }
    }
};

Register_ResultFilter("camStationId", CamStationIdResultFilter)


class CamGenerationDeltaTimeResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cam = dynamic_cast<CaObject*>(object)) {
            const auto genDeltaTime = cam->asn1()->cam.generationDeltaTime;
            fire(this, t, genDeltaTime, details);
        }
    }
};

Register_ResultFilter("camGenerationDeltaTime", CamGenerationDeltaTimeResultFilter)

} // namespace artery
