package artery.inet;

import inet.physicallayer.wireless.common.contract.packetlevel.IPathLoss;

module DistanceSwitchPathLoss like IPathLoss
{
    parameters:
        @display("i=block/control");
        @class(DistanceSwitchPathLoss);
        double thresholdDistance @unit(m);

    submodules:
        near: <> like IPathLoss {
        }

        far: <> like IPathLoss {
        }
}
