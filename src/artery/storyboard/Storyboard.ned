package artery.storyboard;

moduleinterface IStoryboard {}

simple Storyboard like IStoryboard
{
    parameters:
        @display("i=block/blackboard;is=s");
        @class(Storyboard);
        string python = default("storyboard");
        string middlewareModule = default(".middleware");
        string traciModule = default("^.traci");
        string canvas = default("storyboard");
        bool drawConditions = default(true);
}
