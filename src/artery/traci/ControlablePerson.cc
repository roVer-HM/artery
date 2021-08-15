/*
 * ControlablePerson.cc
 *
 *  Created on: Jul 7, 2021
 *      Author: vm-sts
 */

#include <artery/traci/ControlablePerson.h>

traci::PersonController* ControlablePerson::getPersonController(){
    return getController<traci::PersonController>();
}

