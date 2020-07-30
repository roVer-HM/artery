import inet.applications.contract.IApp;

simple BlackIceCentral like IApp
{
    parameters:
        int reportPort = default(9320);
        int queryPort = default(9321);

    gates:
        input socketIn;
        output socketOut;
}
