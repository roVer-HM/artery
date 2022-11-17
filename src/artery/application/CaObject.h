#ifndef ARTERY_CAOBJECT_H_
#define ARTERY_CAOBJECT_H_

#include <artery/application/LdmObject.h>
#include <vanetza/asn1/cam.hpp>

namespace artery
{

class CaObject : public LdmObject<vanetza::asn1::Cam>
{
    using LdmObject<vanetza::asn1::Cam>::LdmObject;
};

} // namespace artery

#endif /* ARTERY_CAOBJECT_H_ */
