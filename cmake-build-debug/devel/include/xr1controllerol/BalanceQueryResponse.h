// Generated by gencpp from file xr1controllerol/BalanceQueryResponse.msg
// DO NOT EDIT!


#ifndef XR1CONTROLLEROL_MESSAGE_BALANCEQUERYRESPONSE_H
#define XR1CONTROLLEROL_MESSAGE_BALANCEQUERYRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace xr1controllerol
{
template <class ContainerAllocator>
struct BalanceQueryResponse_
{
  typedef BalanceQueryResponse_<ContainerAllocator> Type;

  BalanceQueryResponse_()
    : inBLCMode(false)
    , hasIdle(false)
    , hasPassive(false)
    , hasActive(false)  {
    }
  BalanceQueryResponse_(const ContainerAllocator& _alloc)
    : inBLCMode(false)
    , hasIdle(false)
    , hasPassive(false)
    , hasActive(false)  {
  (void)_alloc;
    }



   typedef uint8_t _inBLCMode_type;
  _inBLCMode_type inBLCMode;

   typedef uint8_t _hasIdle_type;
  _hasIdle_type hasIdle;

   typedef uint8_t _hasPassive_type;
  _hasPassive_type hasPassive;

   typedef uint8_t _hasActive_type;
  _hasActive_type hasActive;





  typedef boost::shared_ptr< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> const> ConstPtr;

}; // struct BalanceQueryResponse_

typedef ::xr1controllerol::BalanceQueryResponse_<std::allocator<void> > BalanceQueryResponse;

typedef boost::shared_ptr< ::xr1controllerol::BalanceQueryResponse > BalanceQueryResponsePtr;
typedef boost::shared_ptr< ::xr1controllerol::BalanceQueryResponse const> BalanceQueryResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace xr1controllerol

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'xr1controllerol': ['/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "64124d4b66b6ff17093b36c682f8a41e";
  }

  static const char* value(const ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x64124d4b66b6ff17ULL;
  static const uint64_t static_value2 = 0x093b36c682f8a41eULL;
};

template<class ContainerAllocator>
struct DataType< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xr1controllerol/BalanceQueryResponse";
  }

  static const char* value(const ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool inBLCMode\n\
bool hasIdle\n\
bool hasPassive\n\
bool hasActive\n\
\n\
\n\
";
  }

  static const char* value(const ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.inBLCMode);
      stream.next(m.hasIdle);
      stream.next(m.hasPassive);
      stream.next(m.hasActive);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BalanceQueryResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xr1controllerol::BalanceQueryResponse_<ContainerAllocator>& v)
  {
    s << indent << "inBLCMode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.inBLCMode);
    s << indent << "hasIdle: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.hasIdle);
    s << indent << "hasPassive: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.hasPassive);
    s << indent << "hasActive: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.hasActive);
  }
};

} // namespace message_operations
} // namespace ros

#endif // XR1CONTROLLEROL_MESSAGE_BALANCEQUERYRESPONSE_H
