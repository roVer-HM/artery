package artery.inet.gemv2;

import inet.physicallayer.wireless.common.contract.packetlevel.IPathLoss;

simple NLOSv like IPathLoss
{
    parameters:
        @display("i=block/control");
        @class(gemv2::NLOSv);
        string vehicleIndexModule;
}
