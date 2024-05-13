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

#include "artery/application/StationType.h"
#include "artery/application/PersonMiddlewareEmpty.h"
#include "artery/utility/InitStages.h"
#include "inet/common/ModuleAccess.h"

namespace artery {

Define_Module(PersonMiddlewareEmpty);

void PersonMiddlewareEmpty::initialize(int stage) {
    if (stage == InitStages::Self) {
        // consistency check - to avoid misleading runtime errors
        auto *host = findHost();
        auto mobilityModule = inet::getModuleFromPar<omnetpp::cModule>(
                par("mobilityModule"), host);
        std::string mobilityModuleTypeName = std::string(mobilityModule->getClassName());
        if (mobilityModuleTypeName.find("Vadere") != std::string::npos) {
            throw omnetpp::cRuntimeError(
                    "Mobility module type '%s' is not suitable for this middleware - check your simulation setup! (useVadere parameter and mobilityModule.typename)",
                    mobilityModuleTypeName.c_str());
        }

        // initialize all required parameters
        mMobility = inet::getModuleFromPar<PersonMobility>(
                par("mobilityModule"), host);
        mStationType = vanetza::geonet::StationType::Pedestrian;
        getFacilities().register_const(mMobility);

        Identity identity;
        identity.traci = mMobility->getTraciId();
        identity.application = Identity::randomStationId(getRNG(0));
        emit(Identity::changeSignal,
                Identity::ChangeTraCI | Identity::ChangeStationId, &identity);
    }
    MiddlewareBase::initialize(stage);
}

} /* namespace artery */
