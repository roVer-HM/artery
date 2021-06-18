#ifndef CORE_H_HPQGM1MF
#define CORE_H_HPQGM1MF

#include <omnetpp/cmessage.h>
#include <omnetpp/csimplemodule.h>
#include <omnetpp/simtime.h>
#include <memory>

namespace traci
{

class API;
class Launcher;
class ISubscriptionManager;

class Core : public omnetpp::cSimpleModule
{
public:
    Core();
    virtual ~Core();

    void initialize() override;
    void finish() override;
    void handleMessage(omnetpp::cMessage*) override;
    std::shared_ptr<API> getAPI();

protected:
    virtual void checkVersion();
    virtual void syncTime();

    omnetpp::cMessage* m_connectEvent;
    omnetpp::cMessage* m_updateEvent;
    omnetpp::SimTime m_updateInterval;

    Launcher* m_launcher;
    std::shared_ptr<API> m_traci;
    bool m_stopping;
    ISubscriptionManager* m_subscriptions;
};

} // namespace traci

#endif /* CORE_H_HPQGM1MF */

