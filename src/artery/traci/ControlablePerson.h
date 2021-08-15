/*
 * ControlablePerson.h
 *
 *  Created on: Jul 7, 2021
 *      Author: vm-sts
 */

#ifndef ARTERY_TRACI_CONTROLABLEPERSON_H_
#define ARTERY_TRACI_CONTROLABLEPERSON_H_

#include <artery/traci/ControllableObject.h>
#include "artery/traci/PersonController.h"

class ControlablePerson: public ControllableObject {
public:
    virtual ~ControlablePerson() = default;
    virtual traci::PersonController* getPersonController();
};


#endif /* ARTERY_TRACI_CONTROLABLEPERSON_H_ */
