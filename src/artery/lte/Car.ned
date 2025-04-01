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
        *.interfaceTableModule = default(absPath(".interfaceTable"));  
        ipv4.configurator.typename = "HostAutoConfigurator";
        ipv4.configurator.interfaces = "cellular";

        // LTE NIC
        int masterId @mutable;
        int macNodeId @mutable = default(0); // TODO: this is not a real parameter
        int macCellId @mutable = default(0); // TODO: this is not a real parameter

        //# Apps
        int numApps = default(0);   // no of apps.

        //# Transport layer
        bool hasUdp = default(firstAvailableOrEmpty("Udp") != "");
        bool hasTcp = default(firstAvailableOrEmpty("Tcp", "TcpLwip", "TcpNsc") != "");
        
        //# External Ethernet interfaces
        int numEthInterfaces = default(0);
        string extHostAddress = default("");

    gates:
        input lteRadioIn @directIn;

    submodules:
        //# submodules for network layers (need to be ordered from lower to upper layer)
        cellularNic: <default("LteNicUe")> like ICellularNic {
            @display("p=150,400");
            nodeType = "UE";
        }

        ipv4: <default("Ipv4NetworkLayer")> like INetworkLayer {
            @display("p=150,125");
        }

        udp: <default(firstAvailableOrEmpty("Udp"))> like IUdp if hasUdp {
            parameters:
                @display("p=375,226");
        }
        tcp: <default(firstAvailableOrEmpty("Tcp", "TcpLwip", "TcpNsc"))> like ITcp if hasTcp {
            parameters:
                @display("p=525,226");
        }

        app[numApps]: <> like IApp {
            @display("p=375,76,row,140");
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

        ipv4.ifIn <-- nl.out++;
        ipv4.ifOut --> nl.in++;

        ipv4.transportIn <-- tn.out++;
        ipv4.transportOut --> tn.in++;
        
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
