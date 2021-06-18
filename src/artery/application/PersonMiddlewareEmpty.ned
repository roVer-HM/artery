//
// Artery V2X Simulation Framework
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.application;
import artery.application.MiddlewareBase;

// Middleware for ITS G5 services
simple PersonMiddlewareEmpty extends MiddlewareBase
{
	parameters:
		@class(PersonMiddlewareEmpty);
		@signal[IdentityChanged](type=long);

		string mobilityModule;
}
