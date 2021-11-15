#ifndef ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT
#define ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT

#include "artery/application/CaObject.h"
#include "artery/application/VaObject.h"
#include <omnetpp/simtime.h>
#include <vanetza/asn1/cam.hpp>
#include <cstdint>
#include <functional>
#include <map>
#include <variant>

namespace artery
{

class Timer;

class LocalDynamicMap
{
public:
    using StationID = uint32_t;
    using Cam = vanetza::asn1::Cam;
    using CamPredicate = std::function<bool(const Cam&)>;
    using Vam = vanetza::asn1::Vam;
    using VamPredicate = std::function<bool(const Vam&)>;

    LocalDynamicMap(const Timer&);
    void updateAwareness(const CaObject&);
    void updateAwareness(const VaObject&);
    void dropExpired();
    unsigned count(const CamPredicate&) const;
    unsigned count(const VamPredicate&) const;

private:
    struct AwarenessEntry
    {
        AwarenessEntry(const std::variant<CaObject, VaObject>&, omnetpp::SimTime);
        AwarenessEntry(AwarenessEntry&&) = default;
        AwarenessEntry& operator=(AwarenessEntry&&) = default;

        omnetpp::SimTime expiry;
        std::variant<CaObject, VaObject> object;
    };


    const Timer& mTimer;
    std::map<StationID, AwarenessEntry> mMessages;
};

} // namespace artery

#endif /* ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT */

