/*
 * Artery V2X Simulation Framework
 * Copyright 2017 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef IDENTITY_H_WXAWFSP2
#define IDENTITY_H_WXAWFSP2

#include "artery/application/NetworkInterface.h"
#include <omnetpp/cmodule.h>
#include <omnetpp/cobject.h>
#include <vanetza/geonet/address.hpp>
#include <vanetza/net/mac_address.hpp>
#include <cstdint>
#include <memory>
#include <string>

namespace artery
{

class Identity : public omnetpp::cObject
{
public:
    static const omnetpp::simsignal_t changeSignal;
    enum Changes {
        ChangeNone = 0,
        ChangeTraCI = 1,
        ChangeStationId = 2,
        ChangeGeoNetAddress = 4
    };

    bool update(const Identity&, long changes);
    vanetza::MacAddress mid() const {return geonet.begin()->second.mid();}   // TODO: extend for multiple interfaces

    omnetpp::cModule* host = nullptr; /*< host module, e.g. vehicle node */
    std::string traci; /*< Vehicle ID used by TraCI protocol */
    uint32_t application = 0; /*< ETSI station ID */

    /* NetworkInterface <-> GeoNetworking address mapping */
    std::map<std::shared_ptr<const NetworkInterface>, vanetza::geonet::Address> geonet;

    static uint32_t randomStationId(omnetpp::cRNG*);
    static uint32_t deriveStationId(omnetpp::cModule*, const std::string& basis);
};

} // namespace artery

#endif /* IDENTITY_H_WXAWFSP2 */

