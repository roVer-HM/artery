import inet.applications.contract.IApp;

simple BlackIceWarnerD2D like IApp
{
    parameters:
        int localPort = default(9330);
        int mcastPort = default(9331);
        string mcastAddress = default("224.0.0.10");
        string middlewareModule;
        string interfaceTableModule;
        double warningRadius @unit(m) = default(100 m);
        double warningDuration @unit(s) = default(10 s);

    gates:
        input socketIn;
        output socketOut;
}
