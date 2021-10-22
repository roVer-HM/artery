#ifndef ARTERY_VAOBJECT_H_
#define ARTERY_VAOBJECT_H_

#include <artery/application/LdmObject.h>
#include <vanetza/asn1/vam.hpp>

namespace artery
{

class VaObject : public LdmObject<vanetza::asn1::Vam>
{
    using LdmObject<vanetza::asn1::Vam>::LdmObject;
};

} // namespace artery

#endif /* ARTERY_VAOBJECT_H_ */
