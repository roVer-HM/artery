package artery.lte;

import simu5g.common.CellularNetworkConfigurator;
import inet.node.ethernet.Eth10G;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import simu5g.common.binder.Binder;
import simu5g.common.carrierAggregation.CarrierAggregation;
import simu5g.nodes.eNodeB;
import simu5g.nodes.PgwStandard;
import simu5g.world.radio.LteChannelControl;

network World extends artery.inet.World
{
    parameters:
        int numLteBaseStations = default(1);
        traci.mapper.vehicleType = default("artery.lte.Car");

    submodules:
        channelControl: LteChannelControl {
            parameters:
                @display("p=140,20");
        }

        binder: Binder {
            parameters:
                @display("p=140,60");
        }
        
        carrierAggregation: CarrierAggregation {
            @display("p=50.993748,258.7;is=s");
        }

        configurator: CellularNetworkConfigurator {
            parameters:
                @display("p=60,60");
        }

        eNodeB[numLteBaseStations]: eNodeB {
            parameters:
                mobility.initFromDisplayString = false;
                macNodeId = default(index + 1);
                macCellId = default(index + 1);
                **.eNodeBCount = default(parent.numLteBaseStations);
        }

        pgw: PgwStandard {
            parameters:
                @display("p=220,100");
        }

        server: StandardHost {
            @display("p=220,20;is=n;i=device/server");
        }

        router: Router {
            @display("p=220,60;i=device/smallrouter");
        }

    connections allowunconnected:
        for i = 0..numLteBaseStations-2 {
            eNodeB[i].x2++ <--> Eth10G <--> eNodeB[i+1].x2++;
        }

        for i = 0..numLteBaseStations-1 {
            pgw.pppg++ <--> Eth10G <--> eNodeB[i].ppp;
        }

        server.pppg++ <--> Eth10G <--> router.pppg++;
        router.pppg++ <--> Eth10G <--> pgw.filterGate;
}
