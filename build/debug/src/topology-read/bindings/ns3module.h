/* This file was generated by PyBindGen 0.17.0.868 */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stddef.h>


#if PY_VERSION_HEX < 0x020400F0

#define PyEval_ThreadsInitialized() 1

#define Py_CLEAR(op)				\
        do {                            	\
                if (op) {			\
                        PyObject *tmp = (PyObject *)(op);	\
                        (op) = NULL;		\
                        Py_DECREF(tmp);		\
                }				\
        } while (0)


#define Py_VISIT(op)							\
        do { 								\
                if (op) {						\
                        int vret = visit((PyObject *)(op), arg);	\
                        if (vret)					\
                                return vret;				\
                }							\
        } while (0)

#endif



#if PY_VERSION_HEX < 0x020500F0

typedef int Py_ssize_t;
# define PY_SSIZE_T_MAX INT_MAX
# define PY_SSIZE_T_MIN INT_MIN
typedef inquiry lenfunc;
typedef intargfunc ssizeargfunc;
typedef intobjargproc ssizeobjargproc;

#endif


#ifndef PyVarObject_HEAD_INIT
#define PyVarObject_HEAD_INIT(type, size) \
        PyObject_HEAD_INIT(type) size,
#endif


#if PY_VERSION_HEX >= 0x03000000
typedef void* cmpfunc;
#define PyCObject_FromVoidPtr(a, b) PyCapsule_New(a, NULL, b)
#define PyCObject_AsVoidPtr(a) PyCapsule_GetPointer(a, NULL)
#define PyString_FromString(a) PyBytes_FromString(a)
#define Py_TPFLAGS_CHECKTYPES 0 /* this flag doesn't exist in python 3 */
#endif


#if     __GNUC__ > 2
# define PYBINDGEN_UNUSED(param) param __attribute__((__unused__))
#elif     __GNUC__ > 2 || (__GNUC__ == 2 && __GNUC_MINOR__ > 4)
# define PYBINDGEN_UNUSED(param) __attribute__((__unused__)) param
#else
# define PYBINDGEN_UNUSED(param) param
#endif  /* !__GNUC__ */

#ifndef _PyBindGenWrapperFlags_defined_
#define _PyBindGenWrapperFlags_defined_
typedef enum _PyBindGenWrapperFlags {
   PYBINDGEN_WRAPPER_FLAG_NONE = 0,
   PYBINDGEN_WRAPPER_FLAG_OBJECT_NOT_OWNED = (1<<0),
} PyBindGenWrapperFlags;
#endif


#include "ns3/topology-read-module.h"
#include <ostream>
#include <sstream>
#include <typeinfo>
#include <map>
#include <iostream>
/* --- forward declarations --- */


typedef struct {
    PyObject_HEAD
    ns3::Address *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Address;


extern PyTypeObject *_PyNs3Address_Type;
#define PyNs3Address_Type (*_PyNs3Address_Type)

extern std::map<void*, PyObject*> *_PyNs3Address_wrapper_registry;
#define PyNs3Address_wrapper_registry (*_PyNs3Address_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::AttributeConstructionList *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3AttributeConstructionList;


extern PyTypeObject *_PyNs3AttributeConstructionList_Type;
#define PyNs3AttributeConstructionList_Type (*_PyNs3AttributeConstructionList_Type)

extern std::map<void*, PyObject*> *_PyNs3AttributeConstructionList_wrapper_registry;
#define PyNs3AttributeConstructionList_wrapper_registry (*_PyNs3AttributeConstructionList_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::AttributeConstructionList::Item *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3AttributeConstructionListItem;


extern PyTypeObject *_PyNs3AttributeConstructionListItem_Type;
#define PyNs3AttributeConstructionListItem_Type (*_PyNs3AttributeConstructionListItem_Type)

extern std::map<void*, PyObject*> *_PyNs3AttributeConstructionListItem_wrapper_registry;
#define PyNs3AttributeConstructionListItem_wrapper_registry (*_PyNs3AttributeConstructionListItem_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::CallbackBase *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3CallbackBase;


extern PyTypeObject *_PyNs3CallbackBase_Type;
#define PyNs3CallbackBase_Type (*_PyNs3CallbackBase_Type)

extern std::map<void*, PyObject*> *_PyNs3CallbackBase_wrapper_registry;
#define PyNs3CallbackBase_wrapper_registry (*_PyNs3CallbackBase_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::Hasher *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Hasher;


extern PyTypeObject *_PyNs3Hasher_Type;
#define PyNs3Hasher_Type (*_PyNs3Hasher_Type)

extern std::map<void*, PyObject*> *_PyNs3Hasher_wrapper_registry;
#define PyNs3Hasher_wrapper_registry (*_PyNs3Hasher_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::Ipv4Address *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv4Address;


extern PyTypeObject *_PyNs3Ipv4Address_Type;
#define PyNs3Ipv4Address_Type (*_PyNs3Ipv4Address_Type)

extern std::map<void*, PyObject*> *_PyNs3Ipv4Address_wrapper_registry;
#define PyNs3Ipv4Address_wrapper_registry (*_PyNs3Ipv4Address_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::Ipv4Mask *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv4Mask;


extern PyTypeObject *_PyNs3Ipv4Mask_Type;
#define PyNs3Ipv4Mask_Type (*_PyNs3Ipv4Mask_Type)

extern std::map<void*, PyObject*> *_PyNs3Ipv4Mask_wrapper_registry;
#define PyNs3Ipv4Mask_wrapper_registry (*_PyNs3Ipv4Mask_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::Ipv6Address *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv6Address;


extern PyTypeObject *_PyNs3Ipv6Address_Type;
#define PyNs3Ipv6Address_Type (*_PyNs3Ipv6Address_Type)

extern std::map<void*, PyObject*> *_PyNs3Ipv6Address_wrapper_registry;
#define PyNs3Ipv6Address_wrapper_registry (*_PyNs3Ipv6Address_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::Ipv6Prefix *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv6Prefix;


extern PyTypeObject *_PyNs3Ipv6Prefix_Type;
#define PyNs3Ipv6Prefix_Type (*_PyNs3Ipv6Prefix_Type)

extern std::map<void*, PyObject*> *_PyNs3Ipv6Prefix_wrapper_registry;
#define PyNs3Ipv6Prefix_wrapper_registry (*_PyNs3Ipv6Prefix_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::NodeContainer *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3NodeContainer;


extern PyTypeObject *_PyNs3NodeContainer_Type;
#define PyNs3NodeContainer_Type (*_PyNs3NodeContainer_Type)

extern std::map<void*, PyObject*> *_PyNs3NodeContainer_wrapper_registry;
#define PyNs3NodeContainer_wrapper_registry (*_PyNs3NodeContainer_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::ObjectBase *obj;
    PyObject *inst_dict;
    PyBindGenWrapperFlags flags:8;
} PyNs3ObjectBase;


extern PyTypeObject *_PyNs3ObjectBase_Type;
#define PyNs3ObjectBase_Type (*_PyNs3ObjectBase_Type)

class PyNs3ObjectBase__PythonHelper : public ns3::ObjectBase
{
public:
    PyObject *m_pyself;

    void set_pyobj(PyObject *pyobj)
    {
        Py_XDECREF(m_pyself);
        Py_INCREF(pyobj);
        m_pyself = pyobj;
    }

    virtual ~PyNs3ObjectBase__PythonHelper()
    {
        Py_CLEAR(m_pyself);
    }

};

extern std::map<void*, PyObject*> *_PyNs3ObjectBase_wrapper_registry;
#define PyNs3ObjectBase_wrapper_registry (*_PyNs3ObjectBase_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::ObjectDeleter *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3ObjectDeleter;


extern PyTypeObject *_PyNs3ObjectDeleter_Type;
#define PyNs3ObjectDeleter_Type (*_PyNs3ObjectDeleter_Type)

extern std::map<void*, PyObject*> *_PyNs3ObjectDeleter_wrapper_registry;
#define PyNs3ObjectDeleter_wrapper_registry (*_PyNs3ObjectDeleter_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::SimpleRefCount< ns3::Object, ns3::ObjectBase, ns3::ObjectDeleter > *obj;
    PyObject *inst_dict;
    PyBindGenWrapperFlags flags:8;
} PyNs3SimpleRefCount__Ns3Object_Ns3ObjectBase_Ns3ObjectDeleter;


extern PyTypeObject *_PyNs3SimpleRefCount__Ns3Object_Ns3ObjectBase_Ns3ObjectDeleter_Type;
#define PyNs3SimpleRefCount__Ns3Object_Ns3ObjectBase_Ns3ObjectDeleter_Type (*_PyNs3SimpleRefCount__Ns3Object_Ns3ObjectBase_Ns3ObjectDeleter_Type)



#include <map>
#include <string>
#include <typeinfo>
#if defined(__GNUC__) && __GNUC__ >= 3 && !defined(__clang__)
# include <cxxabi.h>
#endif

#define PBG_TYPEMAP_DEBUG 0

namespace pybindgen {

class TypeMap
{
   std::map<std::string, PyTypeObject *> m_map;

public:

   TypeMap() {}

   void register_wrapper(const std::type_info &cpp_type_info, PyTypeObject *python_wrapper)
   {

#if PBG_TYPEMAP_DEBUG
   std::cerr << "register_wrapper(this=" << this << ", type_name=" << cpp_type_info.name()
             << ", python_wrapper=" << python_wrapper->tp_name << ")" << std::endl;
#endif

       m_map[std::string(cpp_type_info.name())] = python_wrapper;
   }



   PyTypeObject * lookup_wrapper(const std::type_info &cpp_type_info, PyTypeObject *fallback_wrapper)
   {

#if PBG_TYPEMAP_DEBUG
   std::cerr << "lookup_wrapper(this=" << this << ", type_name=" << cpp_type_info.name() << ")" << std::endl;
#endif

       PyTypeObject *python_wrapper = m_map[cpp_type_info.name()];
       if (python_wrapper)
           return python_wrapper;
       else {
#if defined(__GNUC__) && __GNUC__ >= 3 && !defined(__clang__)

           // Get closest (in the single inheritance tree provided by cxxabi.h)
           // registered python wrapper.
           const abi::__si_class_type_info *_typeinfo =
               dynamic_cast<const abi::__si_class_type_info*> (&cpp_type_info);
#if PBG_TYPEMAP_DEBUG
          std::cerr << "  -> looking at C++ type " << _typeinfo->name() << std::endl;
#endif
           while (_typeinfo && (python_wrapper = m_map[std::string(_typeinfo->name())]) == 0) {
               _typeinfo = dynamic_cast<const abi::__si_class_type_info*> (_typeinfo->__base_type);
#if PBG_TYPEMAP_DEBUG
               std::cerr << "  -> looking at C++ type " << _typeinfo->name() << std::endl;
#endif
           }

#if PBG_TYPEMAP_DEBUG
          if (python_wrapper) {
              std::cerr << "  -> found match " << std::endl;
          } else {
              std::cerr << "  -> return fallback wrapper" << std::endl;
          }
#endif

           return python_wrapper? python_wrapper : fallback_wrapper;

#else // non gcc 3+ compilers can only match against explicitly registered classes, not hidden subclasses
           return fallback_wrapper;
#endif
       }
   }
};

}


extern pybindgen::TypeMap *_PyNs3SimpleRefCount__Ns3Object_Ns3ObjectBase_Ns3ObjectDeleter__typeid_map;

#define PyNs3SimpleRefCount__Ns3Object_Ns3ObjectBase_Ns3ObjectDeleter__typeid_map (*_PyNs3SimpleRefCount__Ns3Object_Ns3ObjectBase_Ns3ObjectDeleter__typeid_map)


typedef struct {
    PyObject_HEAD
    ns3::TagBuffer *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3TagBuffer;


extern PyTypeObject *_PyNs3TagBuffer_Type;
#define PyNs3TagBuffer_Type (*_PyNs3TagBuffer_Type)

extern std::map<void*, PyObject*> *_PyNs3TagBuffer_wrapper_registry;
#define PyNs3TagBuffer_wrapper_registry (*_PyNs3TagBuffer_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::TimeWithUnit *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3TimeWithUnit;


extern PyTypeObject *_PyNs3TimeWithUnit_Type;
#define PyNs3TimeWithUnit_Type (*_PyNs3TimeWithUnit_Type)

extern std::map<void*, PyObject*> *_PyNs3TimeWithUnit_wrapper_registry;
#define PyNs3TimeWithUnit_wrapper_registry (*_PyNs3TimeWithUnit_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::TypeId *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3TypeId;


extern PyTypeObject *_PyNs3TypeId_Type;
#define PyNs3TypeId_Type (*_PyNs3TypeId_Type)

extern std::map<void*, PyObject*> *_PyNs3TypeId_wrapper_registry;
#define PyNs3TypeId_wrapper_registry (*_PyNs3TypeId_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::TypeId::AttributeInformation *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3TypeIdAttributeInformation;


extern PyTypeObject *_PyNs3TypeIdAttributeInformation_Type;
#define PyNs3TypeIdAttributeInformation_Type (*_PyNs3TypeIdAttributeInformation_Type)

extern std::map<void*, PyObject*> *_PyNs3TypeIdAttributeInformation_wrapper_registry;
#define PyNs3TypeIdAttributeInformation_wrapper_registry (*_PyNs3TypeIdAttributeInformation_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::TypeId::TraceSourceInformation *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3TypeIdTraceSourceInformation;


extern PyTypeObject *_PyNs3TypeIdTraceSourceInformation_Type;
#define PyNs3TypeIdTraceSourceInformation_Type (*_PyNs3TypeIdTraceSourceInformation_Type)

extern std::map<void*, PyObject*> *_PyNs3TypeIdTraceSourceInformation_wrapper_registry;
#define PyNs3TypeIdTraceSourceInformation_wrapper_registry (*_PyNs3TypeIdTraceSourceInformation_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::empty *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Empty;


extern PyTypeObject *_PyNs3Empty_Type;
#define PyNs3Empty_Type (*_PyNs3Empty_Type)

extern std::map<void*, PyObject*> *_PyNs3Empty_wrapper_registry;
#define PyNs3Empty_wrapper_registry (*_PyNs3Empty_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::int64x64_t *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Int64x64_t;


extern PyTypeObject *_PyNs3Int64x64_t_Type;
#define PyNs3Int64x64_t_Type (*_PyNs3Int64x64_t_Type)

extern std::map<void*, PyObject*> *_PyNs3Int64x64_t_wrapper_registry;
#define PyNs3Int64x64_t_wrapper_registry (*_PyNs3Int64x64_t_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::Object *obj;
    PyObject *inst_dict;
    PyBindGenWrapperFlags flags:8;
} PyNs3Object;


extern PyTypeObject *_PyNs3Object_Type;
#define PyNs3Object_Type (*_PyNs3Object_Type)

class PyNs3Object__PythonHelper : public ns3::Object
{
public:
    PyObject *m_pyself;

    void set_pyobj(PyObject *pyobj)
    {
        Py_XDECREF(m_pyself);
        Py_INCREF(pyobj);
        m_pyself = pyobj;
    }

    virtual ~PyNs3Object__PythonHelper()
    {
        Py_CLEAR(m_pyself);
    }

};


typedef struct {
    PyObject_HEAD
    ns3::Object::AggregateIterator *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3ObjectAggregateIterator;


extern PyTypeObject *_PyNs3ObjectAggregateIterator_Type;
#define PyNs3ObjectAggregateIterator_Type (*_PyNs3ObjectAggregateIterator_Type)

extern std::map<void*, PyObject*> *_PyNs3ObjectAggregateIterator_wrapper_registry;
#define PyNs3ObjectAggregateIterator_wrapper_registry (*_PyNs3ObjectAggregateIterator_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::SimpleRefCount< ns3::AttributeAccessor, ns3::empty, ns3::DefaultDeleter<ns3::AttributeAccessor> > *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3SimpleRefCount__Ns3AttributeAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeAccessor__gt__;


extern PyTypeObject *_PyNs3SimpleRefCount__Ns3AttributeAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeAccessor__gt___Type;
#define PyNs3SimpleRefCount__Ns3AttributeAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeAccessor__gt___Type (*_PyNs3SimpleRefCount__Ns3AttributeAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeAccessor__gt___Type)


extern pybindgen::TypeMap *_PyNs3SimpleRefCount__Ns3AttributeAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeAccessor__gt____typeid_map;

#define PyNs3SimpleRefCount__Ns3AttributeAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeAccessor__gt____typeid_map (*_PyNs3SimpleRefCount__Ns3AttributeAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeAccessor__gt____typeid_map)


typedef struct {
    PyObject_HEAD
    ns3::SimpleRefCount< ns3::AttributeChecker, ns3::empty, ns3::DefaultDeleter<ns3::AttributeChecker> > *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3SimpleRefCount__Ns3AttributeChecker_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeChecker__gt__;


extern PyTypeObject *_PyNs3SimpleRefCount__Ns3AttributeChecker_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeChecker__gt___Type;
#define PyNs3SimpleRefCount__Ns3AttributeChecker_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeChecker__gt___Type (*_PyNs3SimpleRefCount__Ns3AttributeChecker_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeChecker__gt___Type)


extern pybindgen::TypeMap *_PyNs3SimpleRefCount__Ns3AttributeChecker_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeChecker__gt____typeid_map;

#define PyNs3SimpleRefCount__Ns3AttributeChecker_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeChecker__gt____typeid_map (*_PyNs3SimpleRefCount__Ns3AttributeChecker_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeChecker__gt____typeid_map)


typedef struct {
    PyObject_HEAD
    ns3::SimpleRefCount< ns3::AttributeValue, ns3::empty, ns3::DefaultDeleter<ns3::AttributeValue> > *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3SimpleRefCount__Ns3AttributeValue_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeValue__gt__;


extern PyTypeObject *_PyNs3SimpleRefCount__Ns3AttributeValue_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeValue__gt___Type;
#define PyNs3SimpleRefCount__Ns3AttributeValue_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeValue__gt___Type (*_PyNs3SimpleRefCount__Ns3AttributeValue_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeValue__gt___Type)


extern pybindgen::TypeMap *_PyNs3SimpleRefCount__Ns3AttributeValue_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeValue__gt____typeid_map;

#define PyNs3SimpleRefCount__Ns3AttributeValue_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeValue__gt____typeid_map (*_PyNs3SimpleRefCount__Ns3AttributeValue_Ns3Empty_Ns3DefaultDeleter__lt__ns3AttributeValue__gt____typeid_map)


typedef struct {
    PyObject_HEAD
    ns3::SimpleRefCount< ns3::CallbackImplBase, ns3::empty, ns3::DefaultDeleter<ns3::CallbackImplBase> > *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3SimpleRefCount__Ns3CallbackImplBase_Ns3Empty_Ns3DefaultDeleter__lt__ns3CallbackImplBase__gt__;


extern PyTypeObject *_PyNs3SimpleRefCount__Ns3CallbackImplBase_Ns3Empty_Ns3DefaultDeleter__lt__ns3CallbackImplBase__gt___Type;
#define PyNs3SimpleRefCount__Ns3CallbackImplBase_Ns3Empty_Ns3DefaultDeleter__lt__ns3CallbackImplBase__gt___Type (*_PyNs3SimpleRefCount__Ns3CallbackImplBase_Ns3Empty_Ns3DefaultDeleter__lt__ns3CallbackImplBase__gt___Type)


extern pybindgen::TypeMap *_PyNs3SimpleRefCount__Ns3CallbackImplBase_Ns3Empty_Ns3DefaultDeleter__lt__ns3CallbackImplBase__gt____typeid_map;

#define PyNs3SimpleRefCount__Ns3CallbackImplBase_Ns3Empty_Ns3DefaultDeleter__lt__ns3CallbackImplBase__gt____typeid_map (*_PyNs3SimpleRefCount__Ns3CallbackImplBase_Ns3Empty_Ns3DefaultDeleter__lt__ns3CallbackImplBase__gt____typeid_map)


typedef struct {
    PyObject_HEAD
    ns3::SimpleRefCount< ns3::Hash::Implementation, ns3::empty, ns3::DefaultDeleter<ns3::Hash::Implementation> > *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3SimpleRefCount__Ns3HashImplementation_Ns3Empty_Ns3DefaultDeleter__lt__ns3HashImplementation__gt__;


extern PyTypeObject *_PyNs3SimpleRefCount__Ns3HashImplementation_Ns3Empty_Ns3DefaultDeleter__lt__ns3HashImplementation__gt___Type;
#define PyNs3SimpleRefCount__Ns3HashImplementation_Ns3Empty_Ns3DefaultDeleter__lt__ns3HashImplementation__gt___Type (*_PyNs3SimpleRefCount__Ns3HashImplementation_Ns3Empty_Ns3DefaultDeleter__lt__ns3HashImplementation__gt___Type)


extern pybindgen::TypeMap *_PyNs3SimpleRefCount__Ns3HashImplementation_Ns3Empty_Ns3DefaultDeleter__lt__ns3HashImplementation__gt____typeid_map;

#define PyNs3SimpleRefCount__Ns3HashImplementation_Ns3Empty_Ns3DefaultDeleter__lt__ns3HashImplementation__gt____typeid_map (*_PyNs3SimpleRefCount__Ns3HashImplementation_Ns3Empty_Ns3DefaultDeleter__lt__ns3HashImplementation__gt____typeid_map)


typedef struct {
    PyObject_HEAD
    ns3::SimpleRefCount< ns3::TraceSourceAccessor, ns3::empty, ns3::DefaultDeleter<ns3::TraceSourceAccessor> > *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3SimpleRefCount__Ns3TraceSourceAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3TraceSourceAccessor__gt__;


extern PyTypeObject *_PyNs3SimpleRefCount__Ns3TraceSourceAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3TraceSourceAccessor__gt___Type;
#define PyNs3SimpleRefCount__Ns3TraceSourceAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3TraceSourceAccessor__gt___Type (*_PyNs3SimpleRefCount__Ns3TraceSourceAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3TraceSourceAccessor__gt___Type)


extern pybindgen::TypeMap *_PyNs3SimpleRefCount__Ns3TraceSourceAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3TraceSourceAccessor__gt____typeid_map;

#define PyNs3SimpleRefCount__Ns3TraceSourceAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3TraceSourceAccessor__gt____typeid_map (*_PyNs3SimpleRefCount__Ns3TraceSourceAccessor_Ns3Empty_Ns3DefaultDeleter__lt__ns3TraceSourceAccessor__gt____typeid_map)


typedef struct {
    PyObject_HEAD
    ns3::Time *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Time;


extern PyTypeObject *_PyNs3Time_Type;
#define PyNs3Time_Type (*_PyNs3Time_Type)

extern std::map<void*, PyObject*> *_PyNs3Time_wrapper_registry;
#define PyNs3Time_wrapper_registry (*_PyNs3Time_wrapper_registry)

typedef struct {
    PyObject_HEAD
    ns3::TraceSourceAccessor *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3TraceSourceAccessor;


extern PyTypeObject *_PyNs3TraceSourceAccessor_Type;
#define PyNs3TraceSourceAccessor_Type (*_PyNs3TraceSourceAccessor_Type)


typedef struct {
    PyObject_HEAD
    ns3::AttributeAccessor *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3AttributeAccessor;


extern PyTypeObject *_PyNs3AttributeAccessor_Type;
#define PyNs3AttributeAccessor_Type (*_PyNs3AttributeAccessor_Type)


typedef struct {
    PyObject_HEAD
    ns3::AttributeChecker *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3AttributeChecker;


extern PyTypeObject *_PyNs3AttributeChecker_Type;
#define PyNs3AttributeChecker_Type (*_PyNs3AttributeChecker_Type)


typedef struct {
    PyObject_HEAD
    ns3::AttributeValue *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3AttributeValue;


extern PyTypeObject *_PyNs3AttributeValue_Type;
#define PyNs3AttributeValue_Type (*_PyNs3AttributeValue_Type)


typedef struct {
    PyObject_HEAD
    ns3::CallbackChecker *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3CallbackChecker;


extern PyTypeObject *_PyNs3CallbackChecker_Type;
#define PyNs3CallbackChecker_Type (*_PyNs3CallbackChecker_Type)


typedef struct {
    PyObject_HEAD
    ns3::CallbackImplBase *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3CallbackImplBase;


extern PyTypeObject *_PyNs3CallbackImplBase_Type;
#define PyNs3CallbackImplBase_Type (*_PyNs3CallbackImplBase_Type)


typedef struct {
    PyObject_HEAD
    ns3::CallbackValue *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3CallbackValue;


extern PyTypeObject *_PyNs3CallbackValue_Type;
#define PyNs3CallbackValue_Type (*_PyNs3CallbackValue_Type)


typedef struct {
    PyObject_HEAD
    ns3::EmptyAttributeValue *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3EmptyAttributeValue;


extern PyTypeObject *_PyNs3EmptyAttributeValue_Type;
#define PyNs3EmptyAttributeValue_Type (*_PyNs3EmptyAttributeValue_Type)


typedef struct {
    PyObject_HEAD
    ns3::Ipv4AddressChecker *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv4AddressChecker;


extern PyTypeObject *_PyNs3Ipv4AddressChecker_Type;
#define PyNs3Ipv4AddressChecker_Type (*_PyNs3Ipv4AddressChecker_Type)


typedef struct {
    PyObject_HEAD
    ns3::Ipv4AddressValue *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv4AddressValue;


extern PyTypeObject *_PyNs3Ipv4AddressValue_Type;
#define PyNs3Ipv4AddressValue_Type (*_PyNs3Ipv4AddressValue_Type)


typedef struct {
    PyObject_HEAD
    ns3::Ipv4MaskChecker *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv4MaskChecker;


extern PyTypeObject *_PyNs3Ipv4MaskChecker_Type;
#define PyNs3Ipv4MaskChecker_Type (*_PyNs3Ipv4MaskChecker_Type)


typedef struct {
    PyObject_HEAD
    ns3::Ipv4MaskValue *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv4MaskValue;


extern PyTypeObject *_PyNs3Ipv4MaskValue_Type;
#define PyNs3Ipv4MaskValue_Type (*_PyNs3Ipv4MaskValue_Type)


typedef struct {
    PyObject_HEAD
    ns3::Ipv6AddressChecker *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv6AddressChecker;


extern PyTypeObject *_PyNs3Ipv6AddressChecker_Type;
#define PyNs3Ipv6AddressChecker_Type (*_PyNs3Ipv6AddressChecker_Type)


typedef struct {
    PyObject_HEAD
    ns3::Ipv6AddressValue *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv6AddressValue;


extern PyTypeObject *_PyNs3Ipv6AddressValue_Type;
#define PyNs3Ipv6AddressValue_Type (*_PyNs3Ipv6AddressValue_Type)


typedef struct {
    PyObject_HEAD
    ns3::Ipv6PrefixChecker *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv6PrefixChecker;


extern PyTypeObject *_PyNs3Ipv6PrefixChecker_Type;
#define PyNs3Ipv6PrefixChecker_Type (*_PyNs3Ipv6PrefixChecker_Type)


typedef struct {
    PyObject_HEAD
    ns3::Ipv6PrefixValue *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3Ipv6PrefixValue;


extern PyTypeObject *_PyNs3Ipv6PrefixValue_Type;
#define PyNs3Ipv6PrefixValue_Type (*_PyNs3Ipv6PrefixValue_Type)


typedef struct {
    PyObject_HEAD
    ns3::NetDevice *obj;
    PyObject *inst_dict;
    PyBindGenWrapperFlags flags:8;
} PyNs3NetDevice;


extern PyTypeObject *_PyNs3NetDevice_Type;
#define PyNs3NetDevice_Type (*_PyNs3NetDevice_Type)


typedef struct {
    PyObject_HEAD
    ns3::Node *obj;
    PyObject *inst_dict;
    PyBindGenWrapperFlags flags:8;
} PyNs3Node;


extern PyTypeObject *_PyNs3Node_Type;
#define PyNs3Node_Type (*_PyNs3Node_Type)

class PyNs3Node__PythonHelper : public ns3::Node
{
public:
    PyObject *m_pyself;

    void set_pyobj(PyObject *pyobj)
    {
        Py_XDECREF(m_pyself);
        Py_INCREF(pyobj);
        m_pyself = pyobj;
    }

    virtual ~PyNs3Node__PythonHelper()
    {
        Py_CLEAR(m_pyself);
    }

};


typedef struct {
    PyObject_HEAD
    ns3::TimeValue *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3TimeValue;


extern PyTypeObject *_PyNs3TimeValue_Type;
#define PyNs3TimeValue_Type (*_PyNs3TimeValue_Type)


typedef struct {
    PyObject_HEAD
    ns3::TypeIdChecker *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3TypeIdChecker;


extern PyTypeObject *_PyNs3TypeIdChecker_Type;
#define PyNs3TypeIdChecker_Type (*_PyNs3TypeIdChecker_Type)


typedef struct {
    PyObject_HEAD
    ns3::TypeIdValue *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3TypeIdValue;


extern PyTypeObject *_PyNs3TypeIdValue_Type;
#define PyNs3TypeIdValue_Type (*_PyNs3TypeIdValue_Type)


typedef struct {
    PyObject_HEAD
    ns3::AddressChecker *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3AddressChecker;


extern PyTypeObject *_PyNs3AddressChecker_Type;
#define PyNs3AddressChecker_Type (*_PyNs3AddressChecker_Type)


typedef struct {
    PyObject_HEAD
    ns3::AddressValue *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3AddressValue;


extern PyTypeObject *_PyNs3AddressValue_Type;
#define PyNs3AddressValue_Type (*_PyNs3AddressValue_Type)


typedef struct {
    PyObject_HEAD
    ns3::TopologyReaderHelper *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3TopologyReaderHelper;


extern PyTypeObject PyNs3TopologyReaderHelper_Type;

extern std::map<void*, PyObject*> PyNs3TopologyReaderHelper_wrapper_registry;

typedef struct {
    PyObject_HEAD
    ns3::TopologyReader *obj;
    PyObject *inst_dict;
    PyBindGenWrapperFlags flags:8;
} PyNs3TopologyReader;


extern PyTypeObject PyNs3TopologyReader_Type;

class PyNs3TopologyReader__PythonHelper : public ns3::TopologyReader
{
public:
    PyObject *m_pyself;
    PyNs3TopologyReader__PythonHelper()
        : ns3::TopologyReader(), m_pyself(NULL)
        {}


    void set_pyobj(PyObject *pyobj)
    {
        Py_XDECREF(m_pyself);
        Py_INCREF(pyobj);
        m_pyself = pyobj;
    }

    virtual ~PyNs3TopologyReader__PythonHelper()
    {
        Py_CLEAR(m_pyself);
    }


    static PyObject * _wrap_NotifyConstructionCompleted(PyNs3TopologyReader *self);
    inline void NotifyConstructionCompleted__parent_caller()
    { ns3::ObjectBase::NotifyConstructionCompleted(); }

    static PyObject * _wrap_DoInitialize(PyNs3TopologyReader *self);
    inline void DoInitialize__parent_caller()
    { ns3::Object::DoInitialize(); }

    static PyObject * _wrap_NotifyNewAggregate(PyNs3TopologyReader *self);
    inline void NotifyNewAggregate__parent_caller()
    { ns3::Object::NotifyNewAggregate(); }

    static PyObject * _wrap_DoDispose(PyNs3TopologyReader *self);
    inline void DoDispose__parent_caller()
    { ns3::Object::DoDispose(); }

    virtual ns3::NodeContainer Read();

    virtual ns3::TypeId GetInstanceTypeId() const;

    virtual void DoDispose();

    virtual void DoInitialize();

    virtual void NotifyNewAggregate();

    virtual void NotifyConstructionCompleted();

    static ns3::TypeId GetTypeId (void)
    {
      static ns3::TypeId tid = ns3::TypeId ("PyNs3TopologyReader__PythonHelper")
        .SetParent< ns3::TopologyReader > ()
        ;
      return tid;
    }
};

NS_OBJECT_ENSURE_REGISTERED (PyNs3TopologyReader__PythonHelper);


typedef struct {
    PyObject_HEAD
    ns3::TopologyReader::Link *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3TopologyReaderLink;


extern PyTypeObject PyNs3TopologyReaderLink_Type;

extern std::map<void*, PyObject*> PyNs3TopologyReaderLink_wrapper_registry;

typedef struct {
    PyObject_HEAD
    ns3::InetTopologyReader *obj;
    PyObject *inst_dict;
    PyBindGenWrapperFlags flags:8;
} PyNs3InetTopologyReader;


extern PyTypeObject PyNs3InetTopologyReader_Type;

class PyNs3InetTopologyReader__PythonHelper : public ns3::InetTopologyReader
{
public:
    PyObject *m_pyself;
    PyNs3InetTopologyReader__PythonHelper()
        : ns3::InetTopologyReader(), m_pyself(NULL)
        {}


    void set_pyobj(PyObject *pyobj)
    {
        Py_XDECREF(m_pyself);
        Py_INCREF(pyobj);
        m_pyself = pyobj;
    }

    virtual ~PyNs3InetTopologyReader__PythonHelper()
    {
        Py_CLEAR(m_pyself);
    }


    static PyObject * _wrap_NotifyConstructionCompleted(PyNs3InetTopologyReader *self);
    inline void NotifyConstructionCompleted__parent_caller()
    { ns3::ObjectBase::NotifyConstructionCompleted(); }

    static PyObject * _wrap_DoInitialize(PyNs3InetTopologyReader *self);
    inline void DoInitialize__parent_caller()
    { ns3::Object::DoInitialize(); }

    static PyObject * _wrap_NotifyNewAggregate(PyNs3InetTopologyReader *self);
    inline void NotifyNewAggregate__parent_caller()
    { ns3::Object::NotifyNewAggregate(); }

    static PyObject * _wrap_DoDispose(PyNs3InetTopologyReader *self);
    inline void DoDispose__parent_caller()
    { ns3::Object::DoDispose(); }

    virtual ns3::NodeContainer Read();

    virtual ns3::TypeId GetInstanceTypeId() const;

    virtual void DoDispose();

    virtual void DoInitialize();

    virtual void NotifyNewAggregate();

    virtual void NotifyConstructionCompleted();

    static ns3::TypeId GetTypeId (void)
    {
      static ns3::TypeId tid = ns3::TypeId ("PyNs3InetTopologyReader__PythonHelper")
        .SetParent< ns3::InetTopologyReader > ()
        ;
      return tid;
    }
};

NS_OBJECT_ENSURE_REGISTERED (PyNs3InetTopologyReader__PythonHelper);


typedef struct {
    PyObject_HEAD
    ns3::OrbisTopologyReader *obj;
    PyObject *inst_dict;
    PyBindGenWrapperFlags flags:8;
} PyNs3OrbisTopologyReader;


extern PyTypeObject PyNs3OrbisTopologyReader_Type;

class PyNs3OrbisTopologyReader__PythonHelper : public ns3::OrbisTopologyReader
{
public:
    PyObject *m_pyself;
    PyNs3OrbisTopologyReader__PythonHelper()
        : ns3::OrbisTopologyReader(), m_pyself(NULL)
        {}


    void set_pyobj(PyObject *pyobj)
    {
        Py_XDECREF(m_pyself);
        Py_INCREF(pyobj);
        m_pyself = pyobj;
    }

    virtual ~PyNs3OrbisTopologyReader__PythonHelper()
    {
        Py_CLEAR(m_pyself);
    }


    static PyObject * _wrap_NotifyConstructionCompleted(PyNs3OrbisTopologyReader *self);
    inline void NotifyConstructionCompleted__parent_caller()
    { ns3::ObjectBase::NotifyConstructionCompleted(); }

    static PyObject * _wrap_DoInitialize(PyNs3OrbisTopologyReader *self);
    inline void DoInitialize__parent_caller()
    { ns3::Object::DoInitialize(); }

    static PyObject * _wrap_NotifyNewAggregate(PyNs3OrbisTopologyReader *self);
    inline void NotifyNewAggregate__parent_caller()
    { ns3::Object::NotifyNewAggregate(); }

    static PyObject * _wrap_DoDispose(PyNs3OrbisTopologyReader *self);
    inline void DoDispose__parent_caller()
    { ns3::Object::DoDispose(); }

    virtual ns3::NodeContainer Read();

    virtual ns3::TypeId GetInstanceTypeId() const;

    virtual void DoDispose();

    virtual void DoInitialize();

    virtual void NotifyNewAggregate();

    virtual void NotifyConstructionCompleted();

    static ns3::TypeId GetTypeId (void)
    {
      static ns3::TypeId tid = ns3::TypeId ("PyNs3OrbisTopologyReader__PythonHelper")
        .SetParent< ns3::OrbisTopologyReader > ()
        ;
      return tid;
    }
};

NS_OBJECT_ENSURE_REGISTERED (PyNs3OrbisTopologyReader__PythonHelper);


typedef struct {
    PyObject_HEAD
    ns3::RocketfuelTopologyReader *obj;
    PyObject *inst_dict;
    PyBindGenWrapperFlags flags:8;
} PyNs3RocketfuelTopologyReader;


extern PyTypeObject PyNs3RocketfuelTopologyReader_Type;

class PyNs3RocketfuelTopologyReader__PythonHelper : public ns3::RocketfuelTopologyReader
{
public:
    PyObject *m_pyself;
    PyNs3RocketfuelTopologyReader__PythonHelper()
        : ns3::RocketfuelTopologyReader(), m_pyself(NULL)
        {}


    void set_pyobj(PyObject *pyobj)
    {
        Py_XDECREF(m_pyself);
        Py_INCREF(pyobj);
        m_pyself = pyobj;
    }

    virtual ~PyNs3RocketfuelTopologyReader__PythonHelper()
    {
        Py_CLEAR(m_pyself);
    }


    static PyObject * _wrap_NotifyConstructionCompleted(PyNs3RocketfuelTopologyReader *self);
    inline void NotifyConstructionCompleted__parent_caller()
    { ns3::ObjectBase::NotifyConstructionCompleted(); }

    static PyObject * _wrap_DoInitialize(PyNs3RocketfuelTopologyReader *self);
    inline void DoInitialize__parent_caller()
    { ns3::Object::DoInitialize(); }

    static PyObject * _wrap_NotifyNewAggregate(PyNs3RocketfuelTopologyReader *self);
    inline void NotifyNewAggregate__parent_caller()
    { ns3::Object::NotifyNewAggregate(); }

    static PyObject * _wrap_DoDispose(PyNs3RocketfuelTopologyReader *self);
    inline void DoDispose__parent_caller()
    { ns3::Object::DoDispose(); }

    virtual ns3::NodeContainer Read();

    virtual ns3::TypeId GetInstanceTypeId() const;

    virtual void DoDispose();

    virtual void DoInitialize();

    virtual void NotifyNewAggregate();

    virtual void NotifyConstructionCompleted();

    static ns3::TypeId GetTypeId (void)
    {
      static ns3::TypeId tid = ns3::TypeId ("PyNs3RocketfuelTopologyReader__PythonHelper")
        .SetParent< ns3::RocketfuelTopologyReader > ()
        ;
      return tid;
    }
};

NS_OBJECT_ENSURE_REGISTERED (PyNs3RocketfuelTopologyReader__PythonHelper);


typedef struct {
    PyObject_HEAD
    std::map< std::string, std::string > *obj;
} Pystd__map__lt___std__string__std__string___gt__;


typedef struct {
    PyObject_HEAD
    Pystd__map__lt___std__string__std__string___gt__ *container;
    std::map< std::string, std::string >::iterator *iterator;
} Pystd__map__lt___std__string__std__string___gt__Iter;


extern PyTypeObject Pystd__map__lt___std__string__std__string___gt___Type;
extern PyTypeObject Pystd__map__lt___std__string__std__string___gt__Iter_Type;

int _wrap_convert_py2c__std__map__lt___std__string__std__string___gt__(PyObject *arg, std::map< std::string, std::string > *container);
/* --- forward declarations --- */


typedef struct {
    PyObject_HEAD
    ns3::Hash::Implementation *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3HashImplementation;


extern PyTypeObject *_PyNs3HashImplementation_Type;
#define PyNs3HashImplementation_Type (*_PyNs3HashImplementation_Type)

/* --- forward declarations --- */


typedef struct {
    PyObject_HEAD
    ns3::Hash::Function::Fnv1a *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3HashFunctionFnv1a;


extern PyTypeObject *_PyNs3HashFunctionFnv1a_Type;
#define PyNs3HashFunctionFnv1a_Type (*_PyNs3HashFunctionFnv1a_Type)


typedef struct {
    PyObject_HEAD
    ns3::Hash::Function::Hash32 *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3HashFunctionHash32;


extern PyTypeObject *_PyNs3HashFunctionHash32_Type;
#define PyNs3HashFunctionHash32_Type (*_PyNs3HashFunctionHash32_Type)


typedef struct {
    PyObject_HEAD
    ns3::Hash::Function::Hash64 *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3HashFunctionHash64;


extern PyTypeObject *_PyNs3HashFunctionHash64_Type;
#define PyNs3HashFunctionHash64_Type (*_PyNs3HashFunctionHash64_Type)


typedef struct {
    PyObject_HEAD
    ns3::Hash::Function::Murmur3 *obj;
    PyBindGenWrapperFlags flags:8;
} PyNs3HashFunctionMurmur3;


extern PyTypeObject *_PyNs3HashFunctionMurmur3_Type;
#define PyNs3HashFunctionMurmur3_Type (*_PyNs3HashFunctionMurmur3_Type)


int _wrap_convert_py2c__std__string(PyObject *value, std::string *address);

