package traci;

moduleinterface TraCiManager {}

module Manager like TraCiManager
{
    parameters:
        @display("i=block/network2;is=s");
        nodes.coreModule = "^.core";
        nodes.mapperModule = "^.mapper";
        nodes.subscriptionsModule = "^.subscriptions";
        subscriptions.coreModule = "^.core";

        bool ignorePersons = default(true);
        *.ignorePersons = ignorePersons;

    submodules:
        core : <default("Core")> like ICore {
        }

        launcher : <default("ConnectLauncher")> like Launcher {
        }

        subscriptions: <default("BasicSubscriptionManager")> like SubscriptionManager {
        }

        nodes : <default("BasicNodeManager")> like NodeManager {
        }

        mapper: <default("BasicModuleMapper")> like ModuleMapper {
        }
}
