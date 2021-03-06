#ifndef API_H_HBQVASFR
#define API_H_HBQVASFR

#include "traci/sumo/utils/traci/TraCIAPI.h"
#include "traci/Angle.h"
#include "traci/Boundary.h"
#include "traci/GeoPosition.h"
#include "traci/Position.h"
#include "traci/Time.h"
#include <omnetpp/simtime.h>

namespace traci
{

struct ServerEndpoint;

class API : public TraCIAPI
{
public:
    using Version = std::pair<int, std::string>;

    virtual TraCIGeoPosition convertGeo(const TraCIPosition&) const;
    virtual TraCIPosition convert2D(const TraCIGeoPosition&) const;

    void sendFile(std::string path, std::string content);
    void connect(const ServerEndpoint&);
};

} // namespace traci

#endif /* API_H_HBQVASFR */

