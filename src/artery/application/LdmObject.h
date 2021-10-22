#ifndef ARTERY_LDMOBJECT_H_
#define ARTERY_LDMOBJECT_H_

#include <omnetpp.h>
#include <omnetpp/cobject.h>
#include <cassert>
#include <memory>

namespace artery
{

template<class T>
class LdmObject : public omnetpp::cObject
{
public:
    typedef T asn1_obj;

    LdmObject(const LdmObject&) = default;
    LdmObject& operator=(const LdmObject&) = default;

    LdmObject(asn1_obj&& obj) :
        m_asn1_obj_wrapper(std::make_shared<asn1_obj>(std::move(obj)))
    {
    }

    LdmObject& operator=(asn1_obj&& obj)
    {
        m_asn1_obj_wrapper = std::make_shared<asn1_obj>(std::move(obj));
        return *this;
    }

    LdmObject(const asn1_obj& obj) :
        m_asn1_obj_wrapper(std::make_shared<asn1_obj>(obj))
    {
    }

    LdmObject& operator=(const asn1_obj& obj)
    {
        m_asn1_obj_wrapper = std::make_shared<asn1_obj>(obj);
        return *this;
    }

    LdmObject(const std::shared_ptr<const asn1_obj>& ptr) :
        m_asn1_obj_wrapper(ptr)
    {
        assert(m_asn1_obj_wrapper);
    }

    LdmObject& operator=(const std::shared_ptr<const asn1_obj>& ptr)
    {
        m_asn1_obj_wrapper = ptr;
        assert(m_asn1_obj_wrapper);
        return *this;
    }

    const asn1_obj& asn1() const
    {
        return *m_asn1_obj_wrapper;
    }

    std::shared_ptr<const asn1_obj> shared_ptr() const
    {
        assert(m_asn1_obj_wrapper);
        return m_asn1_obj_wrapper;
    }

    omnetpp::cObject* dup() const override
    {
        return new LdmObject { *this };
    }

private:
    std::shared_ptr<const asn1_obj> m_asn1_obj_wrapper;
};

} // namespace artery

#endif /* ARTERY_LDMOBJECT_H_ */
