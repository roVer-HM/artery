//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef ARTERY_APPLICATION_VEHICLEMIDDLEWAREEMPTY_H_
#define ARTERY_APPLICATION_VEHICLEMIDDLEWAREEMPTY_H_

#include "artery/application/Middleware.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"

namespace artery {

class VehicleMiddlewareEmpty : public MiddlewareBase {
public:
    virtual ~VehicleMiddlewareEmpty() = default;
    VehicleMiddlewareEmpty();
    void initialize(int stage) override;
    void finish() override;
protected:
    void initializeStationType(const std::string&);
    void initializeVehicleController(omnetpp::cPar&);
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;

private:
    traci::VehicleController* mVehicleController = nullptr;
    VehicleDataProvider mVehicleDataProvider;
};

} /* namespace artery */

#endif /* ARTERY_APPLICATION_VEHICLEMIDDLEWAREEMPTY_H_ */
