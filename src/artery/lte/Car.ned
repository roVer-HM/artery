package artery.lte;

import inet.applications.contract.IApp;
import inet.networklayer.configurator.ipv4.HostAutoConfigurator;
import inet.networklayer.contract.IRoutingTable;
import inet.networklayer.contract.INetworkLayer;
import inet.transportlayer.tcp.Tcp;
import inet.transportlayer.udp.Udp;
import inet.transportlayer.contract.ITcp;
import inet.transportlayer.contract.IUdp;
import inet.common.MessageDispatcher;
import simu5g.stack.ICellularNic;

module Car extends artery.inet.Car
{
    parameters:
        //# Network Layer specs
        *.routingTableModule = default(absPath(".ipv4.routingTable"));   
        ipv4.configurator.typename = "HostAutoConfigurator";
        // ipv4.configurator.interfaces = "cellular";

        // LTE NIC
        string nicType = default("LteNicUe");
        int macNodeId = default(0);
        int masterId;

        //# Apps
        int numApps = default(0);   // no of apps.

        //# Transport layer
        bool hasUdp = default(firstAvailableOrEmpty("Udp") != "");
        bool hasTcp = default(firstAvailableOrEmpty("Tcp", "TcpLwip", "TcpNsc") != "");

    gates:
        input lteRadioIn @directIn;

    submodules:
        configurator: HostAutoConfigurator {
            @display("p=50,25");
        }

        routingTable: <default("IPv4RoutingTable")> like IRoutingTable {
            @display("p=50,125");
        }

        networkLayer: <default("IPv4NetworkLayer")> like INetworkLayer {
            @display("p=150,125");
        }

        cellularNic: <nicType> like ICellularNic {
            @display("p=150,400");
            nodeType = "UE";
        }

        app[numApps]: <> like IApp {
            @display("p=375,76,row,140");
        }

        udp: <default(firstAvailableOrEmpty("Udp"))> like IUdp if hasUdp {
            parameters:
                @display("p=375,226");
        }
        tcp: <default(firstAvailableOrEmpty("Tcp", "TcpLwip", "TcpNsc"))> like ITcp if hasTcp {
            parameters:
                @display("p=525,226");
        }
        
        // message dispatcher for SAP between application and transport layer
        at: MessageDispatcher {
            parameters:
                @display("p=550,146;b=600,5,,,,1");
        }
        // message dispatcher for SAP between transport and network layer
        tn: MessageDispatcher {
            parameters:
                @display("p=550,296;b=600,5,,,,1");
        }
        // message dispatcher for SAP to link layer
        nl: MessageDispatcher {
            parameters:
                @display("p=550,446;b=600,5,,,,1");
        }

    connections allowunconnected:
        cellularNic.radioIn <-- lteRadioIn;
        
        cellularNic.upperLayerOut --> nl.in++;
        cellularNic.upperLayerIn <-- nl.out++;

        networkLayer.ifIn <-- nl.out++;
        networkLayer.ifOut --> nl.in++;

        networkLayer.transportIn <-- tn.out++;
        networkLayer.transportOut --> tn.in++;
        
        udp.ipOut --> tn.in++ if hasUdp;
        udp.ipIn <-- tn.out++ if hasUdp;

        tcp.ipOut --> tn.in++ if hasTcp;
        tcp.ipIn <-- tn.out++ if hasTcp;

        for i=0..numApps-1 {
            app[i].socketOut --> at.in++;
            app[i].socketIn <-- at.out++;
        }

        at.out++ --> udp.appIn if hasUdp;
        at.in++ <-- udp.appOut if hasUdp;

        at.out++ --> tcp.appIn if hasTcp;
        at.in++ <-- tcp.appOut if hasTcp;
}
