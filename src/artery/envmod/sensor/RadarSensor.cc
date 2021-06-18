/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/sensor/RadarSensor.h"
#if OMNETPP_VERSION >= 0x600
  #include <omnetpp/ccontextswitcher.h>
#endif

using namespace omnetpp;

namespace artery
{

Define_Module(RadarSensor);

const std::string& RadarSensor::getSensorCategory() const
{
    static const std::string category = "Radar";
    return category;
}

} // namespace artery
