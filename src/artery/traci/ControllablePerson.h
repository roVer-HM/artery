#ifndef CONTROLLABLEPERSON_H_DJ96H4LS
#define CONTROLLABLEPERSON_H_DJ96H4LS

#include "artery/traci/PersonController.h"
#include "artery/traci/ControllableObject.h"

class ControllablePerson : public ControllableObject
{
public:
    virtual traci::PersonController* getPersonController() = 0;
    virtual ~ControllablePerson() = default;
};

#endif /* CONTROLLABLEPERSON_H_DJ96H4LS */

