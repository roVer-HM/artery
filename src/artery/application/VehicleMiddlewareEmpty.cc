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

#include <artery/application/VehicleMiddlewareEmpty.h>


#include "artery/application/StationType.h"
#include "artery/application/VehicleMiddleware.h"
#include "artery/traci/ControllableVehicle.h"
#include "artery/traci/MobilityBase.h"
#include "artery/utility/InitStages.h"
#include "inet/common/ModuleAccess.h"

using namespace omnetpp;

namespace artery {

Define_Module(VehicleMiddlewareEmpty);

VehicleMiddlewareEmpty::VehicleMiddlewareEmpty() :
    mVehicleDataProvider(0) // OMNeT++ assigns RNG after construction: set final station ID later
{
}

void VehicleMiddlewareEmpty::initialize(int stage)
{
    if (stage == InitStages::Self) {
        initializeVehicleController(par("mobilityModule"));
        initializeStationType(mVehicleController->getVehicleClass());
        getFacilities().register_const(&mVehicleDataProvider);
        getFacilities().registerConst(static_cast<MovingNodeDataProvider*>(&mVehicleDataProvider));
        mVehicleDataProvider.update(getKinematics(*mVehicleController));

        Identity identity;
        identity.traci = mVehicleController->getVehicleId();
        identity.application = Identity::randomStationId(getRNG(0));
        mVehicleDataProvider.setStationId(identity.application);
        emit(Identity::changeSignal, Identity::ChangeTraCI | Identity::ChangeStationId, &identity);
    }

    MiddlewareBase::initialize(stage);
}

void VehicleMiddlewareEmpty::finish()
{
    MiddlewareBase::finish();
    findHost()->unsubscribe(MobilityBase::stateChangedSignal, this);
}

void VehicleMiddlewareEmpty::initializeStationType(const std::string& vclass)
{
    auto gnStationType = deriveStationTypeFromVehicleClass(vclass);
    mVehicleDataProvider.setStationType(gnStationType);
}

void VehicleMiddlewareEmpty::initializeVehicleController(cPar& mobilityPar)
{
    auto mobility = inet::getModuleFromPar<ControllableVehicle>(mobilityPar, findHost());
    mVehicleController = mobility->getVehicleController();
    ASSERT(mVehicleController);

    getFacilities().register_mutable(mVehicleController);
}

void VehicleMiddlewareEmpty::receiveSignal(cComponent* component, simsignal_t signal, cObject* obj, cObject* details)
{
    if (signal == MobilityBase::stateChangedSignal && mVehicleController) {
        mVehicleDataProvider.update(getKinematics(*mVehicleController));
    }
}

} /* namespace artery */
