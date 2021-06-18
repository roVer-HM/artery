package artery.networking;

simple Router
{
    parameters:
        @class(Router);
        @signal[IdentityChanged](type=long);
        @signal[LinkReception](type=GeoNetPacket);

        @statistic[LinkLatency](source="messageAge(LinkReception)"; unit=s; record=vector?);

        string dccModule;
        string middlewareModule;
        string radioDriverModule;
        string runtimeModule;
        string securityModule;

		// Management Information Base (MIB)
		int itsGnDefaultTrafficClass = default(3);
		bool vanetzaDisableBeaconing = default(false); 
		bool deferInitialBeacon = default(true);
        bool disableBeaconing = default(false);
        bool isMobile = default(true);
        double itsGnBeaconServiceRetransmitTimer @unit(s) = default(3.0s);
    	double itsGnBeaconServiceMaxJitter @unit(s) = default(0.75s);
    gates:
        input radioDriverData;
        input radioDriverProperties;
}
