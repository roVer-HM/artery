package artery.inet;

import inet.linklayer.base.MacProtocolBase;
import inet.linklayer.contract.IMacProtocol;
import inet.linklayer.ieee80211.IIeee80211Mac;
import inet.linklayer.ieee80211.mac.contract.IRx;
import inet.linklayer.ieee80211.mac.contract.ITx;
import inet.linklayer.ieee80211.mac.contract.IDcf;
import inet.linklayer.ieee80211.mac.contract.IDs;
import inet.linklayer.ieee80211.mac.contract.IHcf;
import inet.linklayer.ieee80211.mac.coordinationfunction.Hcf;
import inet.linklayer.ieee80211.mac.coordinationfunction.Dcf;

// VanetMac is almost identical to INET's Ieee80211Mac except for the "hcf" submodule
module VanetMac extends MacProtocolBase like IIeee80211Mac
{
    parameters:
        string mibModule;
        string modeSet @enum("a", "b", "g(erp)", "g(mixed)", "n(mixed-2.4Ghz)", "p", "ac") = default("g(mixed)");
        string fcsMode @enum("declared", "computed") = default("declared");
        string initialRadioMode @enum("off", "sleep", "receiver", "transmitter", "transceiver") = default("receiver");

        int mtu @unit(B) = default(2304B);
        bool qosStation = default(true);

        *.mibModule = default(absPath(this.mibModule));
        *.rxModule = "^.rx";
        *.txModule = "^.tx";

        @display("i=block/layer");
        @class(::inet::ieee80211::Ieee80211Mac);
        @signal[linkBroken](type=inet::Packet); // TODO this signal is only present for the statistic to pass the signal check, to be removed
        @signal[modesetChanged](type=inet::physicallayer::Ieee80211ModeSet);  // artery INET4 upd: was NF_MODESET_CHANGED
        @statistic[packetSentToUpper](title="packets sent to upper layer"; record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
        @statistic[packetSentToLower](title="packets sent to lower layer"; record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
        @statistic[packetReceivedFromUpper](title="packets received from upper layer"; record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
        @statistic[packetReceivedFromLower](title="packets received from lower layer"; record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
        @statistic[linkBroken](title="link breaks"; record=count, vector?; interpolationmode=none);
        @statistic[packetDrop](title="packet drops"; source=packetDropped; record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
        @statistic[packetDropIncorrectlyReceived](title="packet drops: incorrectly received"; source=packetDropReasonIsIncorrectlyReceived(packetDropped); record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
        @statistic[packetDropNotAddressedToUs](title="packet drops: not addressed to us"; source=packetDropReasonIsNotAddressedToUs(packetDropped); record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
        @statistic[packetDropQueueOverflow](title="packet drops: queue overflow"; source=packetDropReasonIsQueueOverflow(packetDropped); record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
        @statistic[packetDropRetryLimitReached](title="packet drops: retry limit reached"; source=packetDropReasonIsRetryLimitReached(packetDropped); record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
        @statistic[packetDropDuplicateDetected](title="packet drops: duplicate detected"; source=packetDropReasonIsDuplicateDetected(packetDropped); record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
        @statistic[packetDropOther](title="packet drops: other"; source=packetDropReasonIsOther(packetDropped); record=count,sum(packetBytes),vector(packetBytes); interpolationmode=none);
    gates:
        input mgmtIn;
        output mgmtOut;
    submodules:
        dcf: <default("Dcf")> like IDcf {
            parameters:
                @display("p=100,100");
        }
        hcf: <default("VanetHcf")> like IHcf if qosStation {
            parameters:
                @display("p=250,100");
        }
        ds: <default("Ds")> like IDs {
            parameters:
                @display("p=400,100");
        }
        rx: <default("Rx")> like IRx {
            parameters:
                @display("p=100,200");
        }
        tx: <default("Tx")> like ITx {
            parameters:
                @display("p=250,200");
        }
}
