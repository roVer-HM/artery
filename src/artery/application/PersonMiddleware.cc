/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2021 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/StationType.h"
#include "artery/application/PersonMiddleware.h"
#include "artery/utility/InitStages.h"
#include "inet/common/ModuleAccess.h"

using namespace omnetpp;

namespace artery
{

Define_Module(PersonMiddleware)

void PersonMiddleware::initialize(int stage)
{
    if (stage == InitStages::Self) {
        findHost()->subscribe(MobilityBase::stateChangedSignal, this);
        initializePersonController(par("mobilityModule"));
        initializeStationType(mPersonController->getNodeClass());

        getFacilities().register_const(&mDataProvider);
        getFacilities().registerConst(static_cast<MovingNodeDataProvider*>(&mDataProvider));
        mDataProvider.update(getKinematics(*mPersonController));


        Identity identity;
        identity.traci = mPersonController->getNodeId();
        identity.application = Identity::randomStationId(getRNG(0));
        mDataProvider.setStationId(identity.application);
        emit(Identity::changeSignal, Identity::ChangeTraCI | Identity::ChangeStationId, &identity);
    }

    Middleware::initialize(stage);
}

void PersonMiddleware::initializeStationType(const std::string& vclass)
{
    auto gnStationType = deriveStationTypeFromVehicleClass(vclass);
    setStationType(gnStationType);
    mDataProvider.setStationType(gnStationType);
}

void PersonMiddleware::initializePersonController(cPar& mobilityPar)
{
    auto mobility = inet::getModuleFromPar<ControlablePerson>(mobilityPar, findHost());
    mPersonController = mobility->getPersonController();
    ASSERT(mPersonController);

    getFacilities().register_const(mobility);
    getFacilities().register_mutable(mobility->getControllerBase());
    getFacilities().register_mutable(mPersonController);
}

void PersonMiddleware::receiveSignal(cComponent* component, simsignal_t signal, cObject* obj, cObject* details)
{
    if (signal == MobilityBase::stateChangedSignal && mPersonController) {
        mDataProvider.update(getKinematics(*mPersonController));
    }
}


} // namespace artery

