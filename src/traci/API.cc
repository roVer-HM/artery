#include "traci/API.h"
#include "traci/Launcher.h"
#include <thread>

namespace traci
{

TraCIGeoPosition API::convertGeo(const TraCIPosition& pos) const
{
    libsumo::TraCIPosition result = simulation.convertGeo(pos.x, pos.y, false);
    TraCIGeoPosition geo;
    geo.longitude = result.x;
    geo.latitude = result.y;
    return geo;
}

TraCIPosition API::convert2D(const TraCIGeoPosition& pos) const
{
    return simulation.convertGeo(pos.longitude, pos.latitude, true);
}

void API::sendFile(std::string path, std::string content){
    tcpip::Storage outMsg;

    int length = 5 + 1 + 4 + static_cast<int>(path.length()) + 4 +
                 static_cast<int>(content.length());
    outMsg.writeUnsignedByte(0);  // 5 byte length field
    outMsg.writeInt(length);
    outMsg.writeByte(0x75);   // SEND_FILE cmd (veins)
    outMsg.writeString(path); // raw string (no type here)
    outMsg.writeString(content);

    mySocket->sendExact(outMsg);
    tcpip::Storage inMsg;
    check_resultState(inMsg, 0x75);
}

void API::connect(const ServerEndpoint& endpoint)
{
    const unsigned max_tries = endpoint.retry ? 10 : 0;
    unsigned tries = 0;
    auto sleep = std::chrono::milliseconds(500);

    while (true) {
        try {
            TraCIAPI::connect(endpoint.hostname, endpoint.port);
            return;
        } catch (tcpip::SocketException&) {
            if (++tries < max_tries) {
                std::this_thread::sleep_for(sleep);
                sleep *= 2;
            } else {
                throw;
            }
        }
    }
}

} // namespace traci
