package artery.application;

simple VaService like ItsG5Service
{
    parameters:
        @signal[VamReceived](type=VaObject);
        @signal[VamSent](type=VaObject);
        
        @statistic[reception](source=VamReceived;record=vector(vamStationId)?,vector(vamGenerationDeltaTime)?);
        @statistic[transmission](source=VamSent;record=vector(vamStationId)?,vector(vamGenerationDeltaTime)?);
        	
       	// enable Dcc transmission interval restrictions
        bool withDccRestriction = default(true);
        
        // Vam generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(5.0s);
        
        // Generation interval for VAMs containing the low frequency container.
        double genLfContainer @unit(s) = default(2.0s);
        
        // Threshold for reference position change between two consecutive VAMs.
        double referencePositionThreshold @unit(m) = default(4.0m);
        
        // Threshold for speed change between two consecutive VAMs.
        double speedThreshold @unit(mps) = default(0.5mps);
        
        // Threshold for orientation change between two consecutive VAMs.
        double orientationThreshold  = default(4.0);
        
        //  Number of times a VRU ITS-S shall skip the generation of an individual VAM if the conditions for redundancy mitigation are satisfied.
        int numSkipVamRedundancy = default(5);
        
        // Indication about parallel activity of the VRU.
        int vruDeviceUsage = default(2);
        
        //  Threshold for trajectory interception of the  VRU and other road users between two consecutive VAMs.
        double trajectoryInterceptionThreshold = default(0.1);
        
        // Minimum safe lateral distance between the VRU and other road users.
        double minLatDistance @unit(m) = default(5.0m);
        
        // Minimum safe longitudinal distance between the VRU and other road users.
        double minLongDistance @unit(m) = default(5.0m);
        
        // Minimum safe vertical distance between the VRU and other road users.
        double minVertDistance @unit(m) = default(5.0m);
        
}