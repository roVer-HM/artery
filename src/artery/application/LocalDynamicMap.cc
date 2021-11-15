#include "artery/application/LocalDynamicMap.h"
#include "artery/application/Timer.h"
#include <omnetpp/csimulation.h>
#include <cassert>
#include <algorithm>

namespace artery
{

LocalDynamicMap::LocalDynamicMap(const Timer& timer) :
    mTimer(timer)
{
}

void LocalDynamicMap::updateAwareness(const CaObject& obj)
{
    const vanetza::asn1::Cam& msg = obj.asn1();

    static const omnetpp::SimTime lifetime { 1100, omnetpp::SIMTIME_MS };
    auto tai = mTimer.reconstructMilliseconds(msg->cam.generationDeltaTime);
    const omnetpp::SimTime expiry = mTimer.getTimeFor(tai) + lifetime;

    const auto now = omnetpp::simTime();
    if (expiry < now || expiry > now + 2 * lifetime) {
        EV_STATICCONTEXT
        EV_WARN << "Expiry of received CAM is out of bounds";
        return;
    }

    AwarenessEntry entry(obj, expiry);
    auto found = mMessages.find(msg->header.stationID);
    if (found != mMessages.end()) {
        found->second = std::move(entry);
    } else {
        mMessages.emplace(msg->header.stationID, std::move(entry));
    }
}

void LocalDynamicMap::updateAwareness(const VaObject& obj)
{
    const vanetza::asn1::Vam& msg = obj.asn1();

    static const omnetpp::SimTime lifetime { 1100, omnetpp::SIMTIME_MS };
    auto tai = mTimer.reconstructMilliseconds(msg->vam.generationDeltaTime);
    const omnetpp::SimTime expiry = mTimer.getTimeFor(tai) + lifetime;

    const auto now = omnetpp::simTime();
    if (expiry < now || expiry > now + 2 * lifetime) {
        EV_STATICCONTEXT
        EV_WARN << "Expiry of received VAM is out of bounds";
        return;
    }

    AwarenessEntry entry(obj, expiry);
    auto found = mMessages.find(msg->header.stationID);
    if (found != mMessages.end()) {
        found->second = std::move(entry);
    } else {
        mMessages.emplace(msg->header.stationID, std::move(entry));
    }
}

void LocalDynamicMap::dropExpired()
{
    const auto now = omnetpp::simTime();
    for (auto it = mMessages.begin(); it != mMessages.end();) {
        if (it->second.expiry < now) {
            it = mMessages.erase(it);
        } else {
            ++it;
        }
    }

}

unsigned LocalDynamicMap::count(const CamPredicate& predicate) const
{
    return std::count_if(mMessages.begin(), mMessages.end(),
            [&predicate](const std::pair<const StationID, AwarenessEntry>& map_entry) {
                if(!map_entry.second.object.index()){
                    CaObject caObj = std::get<CaObject>(map_entry.second.object);
                    const Cam& cam = caObj.asn1();
                    return predicate(cam);
                }
            });
}

unsigned LocalDynamicMap::count(const VamPredicate& predicate) const
{
    return std::count_if(mMessages.begin(), mMessages.end(),
            [&predicate](const std::pair<const StationID, AwarenessEntry>& map_entry) {
                if(map_entry.second.object.index()){
                    VaObject vaObj = std::get<VaObject>(map_entry.second.object);
                    const Vam& vam = vaObj.asn1();
                    return predicate(vam);
                }
            });
}

LocalDynamicMap::AwarenessEntry::AwarenessEntry(const std::variant<CaObject, VaObject>& obj, omnetpp::SimTime t) :
    expiry(t), object(obj)
{
}


} // namespace artery
