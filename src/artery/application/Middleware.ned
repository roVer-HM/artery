//
// Artery V2X Simulation Framework
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.application;

moduleinterface IMiddleware
{
}

simple MiddlewareBase like IMiddleware {
		@class(MiddlewareBase);
		@signal[IdentityUpdated](type=artery::Identity);
		@signal[IdentityRemoved](type=artery::Identity);

}

// Middleware for ITS G5 services
simple Middleware extends MiddlewareBase like IMiddleware
{
	parameters:
		@class(Middleware);
		@signal[StoryboardSignal](type=StoryboardSignal);

		double updateInterval = default(0.1s) @unit(s);
		string datetime;
		xml services;
		xml mcoPolicy = default(xml("<mco default=\"CCH\" />"));

		string positionProviderModule = default(".vanetza.position");
}
