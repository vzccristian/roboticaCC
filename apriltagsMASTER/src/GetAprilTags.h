// **********************************************************************
//
// Copyright (c) 2003-2013 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.5.1
//
// <auto-generated>
//
// Generated from file `GetAprilTags.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef ____GetAprilTags_h__
#define ____GetAprilTags_h__

#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/Outgoing.h>
#include <Ice/OutgoingAsync.h>
#include <Ice/Incoming.h>
#include <Ice/Direct.h>
#include <IceUtil/ScopedArray.h>
#include <IceUtil/Optional.h>
#include <Ice/StreamF.h>
#include <Ice/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 305
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

namespace IceProxy
{

namespace RoboCompGetAprilTags
{

class GetAprilTags;
void __read(::IceInternal::BasicStream*, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompGetAprilTags::GetAprilTags>&);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompGetAprilTags::GetAprilTags*);

}

}

namespace RoboCompGetAprilTags
{

class GetAprilTags;
bool operator==(const GetAprilTags&, const GetAprilTags&);
bool operator<(const GetAprilTags&, const GetAprilTags&);
::Ice::Object* upCast(::RoboCompGetAprilTags::GetAprilTags*);
typedef ::IceInternal::Handle< ::RoboCompGetAprilTags::GetAprilTags> GetAprilTagsPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompGetAprilTags::GetAprilTags> GetAprilTagsPrx;
void __patch(GetAprilTagsPtr&, const ::Ice::ObjectPtr&);

}

namespace RoboCompGetAprilTags
{

struct marca
{
    ::Ice::Int id;
    ::Ice::Float tx;
    ::Ice::Float ty;
    ::Ice::Float tz;
    ::Ice::Float rx;
    ::Ice::Float ry;
    ::Ice::Float rz;
};

typedef ::std::vector< ::RoboCompGetAprilTags::marca> listaMarcas;

}

namespace Ice
{
template<>
struct StreamableTraits< ::RoboCompGetAprilTags::marca>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 28;
    static const bool fixedLength = true;
};

template<class S>
struct StreamWriter< ::RoboCompGetAprilTags::marca, S>
{
    static void write(S* __os, const ::RoboCompGetAprilTags::marca& v)
    {
        __os->write(v.id);
        __os->write(v.tx);
        __os->write(v.ty);
        __os->write(v.tz);
        __os->write(v.rx);
        __os->write(v.ry);
        __os->write(v.rz);
    }
};

template<class S>
struct StreamReader< ::RoboCompGetAprilTags::marca, S>
{
    static void read(S* __is, ::RoboCompGetAprilTags::marca& v)
    {
        __is->read(v.id);
        __is->read(v.tx);
        __is->read(v.ty);
        __is->read(v.tz);
        __is->read(v.rx);
        __is->read(v.ry);
        __is->read(v.rz);
    }
};

}

namespace RoboCompGetAprilTags
{

class Callback_GetAprilTags_checkMarcas_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_GetAprilTags_checkMarcas_Base> Callback_GetAprilTags_checkMarcasPtr;

}

namespace IceProxy
{

namespace RoboCompGetAprilTags
{

class GetAprilTags : virtual public ::IceProxy::Ice::Object
{
public:

    ::RoboCompGetAprilTags::listaMarcas checkMarcas()
    {
        return checkMarcas(0);
    }
    ::RoboCompGetAprilTags::listaMarcas checkMarcas(const ::Ice::Context& __ctx)
    {
        return checkMarcas(&__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_checkMarcas(const ::IceInternal::Function<void (const ::RoboCompGetAprilTags::listaMarcas&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_checkMarcas(0, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_checkMarcas(const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_checkMarcas(0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_checkMarcas(const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::RoboCompGetAprilTags::listaMarcas&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_checkMarcas(&__ctx, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_checkMarcas(const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_checkMarcas(&__ctx, ::Ice::newCallback(__completed, __sent));
    }
    
private:

    ::Ice::AsyncResultPtr __begin_checkMarcas(const ::Ice::Context* __ctx, const ::IceInternal::Function<void (const ::RoboCompGetAprilTags::listaMarcas&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception, const ::IceInternal::Function<void (bool)>& __sent)
    {
        class Cpp11CB : public ::IceInternal::Cpp11FnCallbackNC
        {
        public:

            Cpp11CB(const ::std::function<void (const ::RoboCompGetAprilTags::listaMarcas&)>& responseFunc, const ::std::function<void (const ::Ice::Exception&)>& exceptionFunc, const ::std::function<void (bool)>& sentFunc) :
                ::IceInternal::Cpp11FnCallbackNC(exceptionFunc, sentFunc),
                _response(responseFunc)
            {
                CallbackBase::checkCallback(true, responseFunc || exceptionFunc != nullptr);
            }

            virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
            {
                ::RoboCompGetAprilTags::GetAprilTagsPrx __proxy = ::RoboCompGetAprilTags::GetAprilTagsPrx::uncheckedCast(__result->getProxy());
                ::RoboCompGetAprilTags::listaMarcas __ret;
                try
                {
                    __ret = __proxy->end_checkMarcas(__result);
                }
                catch(::Ice::Exception& ex)
                {
                    Cpp11FnCallbackNC::__exception(__result, ex);
                    return;
                }
                if(_response != nullptr)
                {
                    _response(__ret);
                }
            }
        
        private:
            
            ::std::function<void (const ::RoboCompGetAprilTags::listaMarcas&)> _response;
        };
        return begin_checkMarcas(__ctx, new Cpp11CB(__response, __exception, __sent));
    }
    
public:
#endif

    ::Ice::AsyncResultPtr begin_checkMarcas()
    {
        return begin_checkMarcas(0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_checkMarcas(const ::Ice::Context& __ctx)
    {
        return begin_checkMarcas(&__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_checkMarcas(const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_checkMarcas(0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_checkMarcas(const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_checkMarcas(&__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_checkMarcas(const ::RoboCompGetAprilTags::Callback_GetAprilTags_checkMarcasPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_checkMarcas(0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_checkMarcas(const ::Ice::Context& __ctx, const ::RoboCompGetAprilTags::Callback_GetAprilTags_checkMarcasPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_checkMarcas(&__ctx, __del, __cookie);
    }

    ::RoboCompGetAprilTags::listaMarcas end_checkMarcas(const ::Ice::AsyncResultPtr&);
    
private:

    ::RoboCompGetAprilTags::listaMarcas checkMarcas(const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_checkMarcas(const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_adapterId(const ::std::string& __id) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_locatorCacheTimeout(int __timeout) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_connectionCached(bool __cached) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_secure(bool __secure) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_preferSecure(bool __preferSecure) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_collocationOptimized(bool __co) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_twoway() const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_twoway().get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_oneway() const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_oneway().get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_batchOneway() const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_datagram() const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_datagram().get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_batchDatagram() const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_compress(bool __compress) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_timeout(int __timeout) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_connectionId(const ::std::string& __id) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<GetAprilTags> ice_encodingVersion(const ::Ice::EncodingVersion& __v) const
    {
        return dynamic_cast<GetAprilTags*>(::IceProxy::Ice::Object::ice_encodingVersion(__v).get());
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

namespace IceDelegate
{

namespace RoboCompGetAprilTags
{

class GetAprilTags : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual ::RoboCompGetAprilTags::listaMarcas checkMarcas(const ::Ice::Context*, ::IceInternal::InvocationObserver&) = 0;
};

}

}

namespace IceDelegateM
{

namespace RoboCompGetAprilTags
{

class GetAprilTags : virtual public ::IceDelegate::RoboCompGetAprilTags::GetAprilTags,
                     virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual ::RoboCompGetAprilTags::listaMarcas checkMarcas(const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace IceDelegateD
{

namespace RoboCompGetAprilTags
{

class GetAprilTags : virtual public ::IceDelegate::RoboCompGetAprilTags::GetAprilTags,
                     virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual ::RoboCompGetAprilTags::listaMarcas checkMarcas(const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace RoboCompGetAprilTags
{

class GetAprilTags : virtual public ::Ice::Object
{
public:

    typedef GetAprilTagsPrx ProxyType;
    typedef GetAprilTagsPtr PointerType;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::RoboCompGetAprilTags::listaMarcas checkMarcas(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___checkMarcas(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

protected:
    virtual void __writeImpl(::IceInternal::BasicStream*) const;
    virtual void __readImpl(::IceInternal::BasicStream*);
    #ifdef __SUNPRO_CC
    using ::Ice::Object::__writeImpl;
    using ::Ice::Object::__readImpl;
    #endif
};

inline bool operator==(const GetAprilTags& l, const GetAprilTags& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

inline bool operator<(const GetAprilTags& l, const GetAprilTags& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

}

namespace RoboCompGetAprilTags
{

template<class T>
class CallbackNC_GetAprilTags_checkMarcas : public Callback_GetAprilTags_checkMarcas_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const ::RoboCompGetAprilTags::listaMarcas&);

    CallbackNC_GetAprilTags_checkMarcas(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompGetAprilTags::GetAprilTagsPrx __proxy = ::RoboCompGetAprilTags::GetAprilTagsPrx::uncheckedCast(__result->getProxy());
        ::RoboCompGetAprilTags::listaMarcas __ret;
        try
        {
            __ret = __proxy->end_checkMarcas(__result);
        }
        catch(::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::__exception(__result, ex);
            return;
        }
        if(response)
        {
            (::IceInternal::CallbackNC<T>::callback.get()->*response)(__ret);
        }
    }

    Response response;
};

template<class T> Callback_GetAprilTags_checkMarcasPtr
newCallback_GetAprilTags_checkMarcas(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompGetAprilTags::listaMarcas&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_GetAprilTags_checkMarcas<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_GetAprilTags_checkMarcasPtr
newCallback_GetAprilTags_checkMarcas(T* instance, void (T::*cb)(const ::RoboCompGetAprilTags::listaMarcas&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_GetAprilTags_checkMarcas<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_GetAprilTags_checkMarcas : public Callback_GetAprilTags_checkMarcas_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const ::RoboCompGetAprilTags::listaMarcas&, const CT&);

    Callback_GetAprilTags_checkMarcas(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompGetAprilTags::GetAprilTagsPrx __proxy = ::RoboCompGetAprilTags::GetAprilTagsPrx::uncheckedCast(__result->getProxy());
        ::RoboCompGetAprilTags::listaMarcas __ret;
        try
        {
            __ret = __proxy->end_checkMarcas(__result);
        }
        catch(::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::__exception(__result, ex);
            return;
        }
        if(response)
        {
            (::IceInternal::Callback<T, CT>::callback.get()->*response)(__ret, CT::dynamicCast(__result->getCookie()));
        }
    }

    Response response;
};

template<class T, typename CT> Callback_GetAprilTags_checkMarcasPtr
newCallback_GetAprilTags_checkMarcas(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompGetAprilTags::listaMarcas&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_GetAprilTags_checkMarcas<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_GetAprilTags_checkMarcasPtr
newCallback_GetAprilTags_checkMarcas(T* instance, void (T::*cb)(const ::RoboCompGetAprilTags::listaMarcas&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_GetAprilTags_checkMarcas<T, CT>(instance, cb, excb, sentcb);
}

}

#endif