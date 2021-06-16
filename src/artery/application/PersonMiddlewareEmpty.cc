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



void PersonMiddlewareEmpty::initialize(int stage)
{
if (stage == InitStages::Self) {
    mMobility = inet::getModuleFromPar<PersonMobility>(par("mobilityModule"), findHost());
    mStationType = vanetza::geonet::StationType::Pedestrian;
    getFacilities().register_const(mMobility);

    Identity identity;
    identity.traci = mMobility->getPersonId();
    identity.application = Identity::randomStationId(getRNG(0));
    emit(Identity::changeSignal, Identity::ChangeTraCI | Identity::ChangeStationId, &identity);
}
MiddlewareBase::initialize(stage);
}

} /* namespace artery */
