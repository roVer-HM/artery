/*
 * MovingObjectSink.h
 *
 *  Created on: Jun 11, 2021
 *      Author: vm-sts
 */

#pragma once
#include <memory>

namespace traci
{

class API;
class VariableCache;

class MovingObjectSink
{
public:
    virtual void initializeSink(std::shared_ptr<API> api, const std::string& id, const Boundary&, std::shared_ptr<VariableCache> cache) = 0;
    virtual void initializeObject(const TraCIPosition& angel, TraCIAngle angle, double speed) = 0;
    virtual void updateObject(const TraCIPosition& pos, TraCIAngle angle, double speed) = 0;
    virtual ~MovingObjectSink() = default;
};

} // namespace traci
