#ifndef CONTROLLABLEVEHICLE_H_DJ96H4LS
#define CONTROLLABLEVEHICLE_H_DJ96H4LS

#include "artery/traci/VehicleController.h"
#include "artery/traci/ControllableObject.h"

class ControllableVehicle: public ControllableObject
{
public:
    virtual traci::VehicleController* getVehicleController() = 0;
    virtual ~ControllableVehicle() = default;
};

#endif /* CONTROLLABLEVEHICLE_H_DJ96H4LS */
