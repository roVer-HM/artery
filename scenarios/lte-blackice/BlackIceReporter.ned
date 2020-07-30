import inet.applications.contract.IApp;

simple BlackIceReporter like IApp
{
    parameters:
        int centralPort = default(9320);
        string centralAddress;
        string middlewareModule;

    gates:
        input socketIn;
        output socketOut;
}
