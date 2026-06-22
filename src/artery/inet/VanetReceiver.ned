package artery.inet;

import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211ScalarRadio;

module VanetReceiver extends Ieee80211ScalarRadio
{
    parameters:
        @class(VanetReceiver);
        double captureThreshold @unit(dB) = default(6 dB);
}
