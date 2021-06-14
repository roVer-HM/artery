/*
 * TraCIApiProvider.h
 *
 *  Created on: Aug 6, 2020
 *      Author: sts
 */

#pragma once

#include "traci/API.h"

namespace traci {

/**
 * create appropriate TraCI API
 */
class TraCIApiProvider {

public:

    virtual std::shared_ptr<API> createAPI() = 0;

};

} /* namespace traci */

