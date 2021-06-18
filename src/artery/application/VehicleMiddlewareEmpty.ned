//
// Artery V2X Simulation Framework
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.application;

// Middleware for ITS G5 services
simple VehicleMiddlewareEmpty extends MiddlewareBase
{
	parameters:
		@class(VehicleMiddlewareEmpty);
		@signal[IdentityChanged](type=long);

		string mobilityModule;
}
