#ifndef ARTERY_CALCULATIONS_H_D4YPLQR
#define ARTERY_CALCULATIONS_H_D4YPLQR

#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/asn1/its/SpeedValue.h>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>


namespace artery
{

using namespace omnetpp;

constexpr auto microdegree = vanetza::units::degree * boost::units::si::micro;
constexpr auto decidegree = vanetza::units::degree * boost::units::si::deci;
constexpr auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
constexpr auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
    boost::units::quantity<U> v { q };
    return std::round(v.value());
}

inline SpeedValue_t buildSpeedValue(const vanetza::units::Velocity& v)
{
    static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
    static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };

    SpeedValue_t speed = SpeedValue_unavailable;
    if (v >= upper) {
        speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
    } else if (v >= lower) {
        speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
    }
    return speed;
}

} // namespace artery

#endif /* ARTERY_CALCULATIONS_H_D4YPLQR */
