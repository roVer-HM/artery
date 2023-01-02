//
// Created by rupp on 02.01.23.
//

#ifndef CROWNET_VAM_VACLUSTERHELPER_H
#define CROWNET_VAM_VACLUSTERHELPER_H

#include <vanetza/asn1/vam.hpp>
#include <math.h>
#include <algorithm>

namespace artery
{
namespace cluster
{

class Velocity
{
public:
    Velocity(double vx, double vy) {
        x = vx;
        y = vy;
        valid = true;
    }

    Velocity(double vx, double vy, bool isValid) {
        x = vx;
        y = vy;
        valid = isValid;
    }


    Velocity operator -(const Velocity& other) const {
        return Velocity(x - other.x, y - other.y);
    }

    double dotProduct(const Velocity& other) const {
        return x * other.x + y * other.y;
    }

    double mag() const {
        return sqrt(pow(x, 2.0) + pow(y, 2.0));
    }

    double getAngleBetween(const Velocity& other) const {
        return acos(dotProduct(other)/(mag() * other.mag()));
    }

    bool isValid() const {
        return valid;
    }

    static Velocity invalid() {
        return Velocity(-1.0, -1.0, false);
    }

private:
    double x;
    double y;
    bool valid;
};

inline double _deg2rad(double angle)
{
    return (angle * M_PI) / 180.0;
}

Velocity getVamVelocity(const vanetza::asn1::Vam *message)
{
    auto hfc = (*message)->vam.vamParameters.vruHighFrequencyContainer;

    if (hfc == nullptr) {
        return Velocity::invalid();
    }

    long speed = hfc->speed.speedValue;
    long heading = hfc->heading.headingValue;

    // Convert 0.1 deg to radians
    double headingRad = _deg2rad(heading / 10.0);

    Velocity vamVelocity{
        sin(headingRad) * speed / 100.0,
        cos(headingRad) * speed / 100.0
    };

    return vamVelocity;
}

double getVeloDifference(const Velocity v1, const Velocity v2) {
    if (!v1.isValid() || !v2.isValid()) {
        return -1;
    }
    double speedDifference = 1 - std::min(v1.mag(), v2.mag()) / std::max(v1.mag(), v2.mag());
    double angleDifference = v1.getAngleBetween(v2) / 180;

    return std::max(speedDifference, angleDifference);

}


}
}

#endif  // CROWNET_VAM_VACLUSTERHELPER_H
