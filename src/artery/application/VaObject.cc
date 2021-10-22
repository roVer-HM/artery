#include <artery/application/VaObject.h>

namespace artery
{

using namespace vanetza::asn1;

Register_Abstract_Class(VaObject)

using namespace omnetpp;

class VamStationIdResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto vam = dynamic_cast<VaObject*>(object)) {
            const auto id = vam->asn1()->header.stationID;
            fire(this, t, id, details);
        }
    }
};

Register_ResultFilter("vamStationId", VamStationIdResultFilter)


class VamGenerationDeltaTimeResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto vam = dynamic_cast<VaObject*>(object)) {
            const auto genDeltaTime = vam->asn1()->vam.generationDeltaTime;
            fire(this, t, genDeltaTime, details);
        }
    }
};

Register_ResultFilter("vamGenerationDeltaTime", VamGenerationDeltaTimeResultFilter)




} // namespace artery
