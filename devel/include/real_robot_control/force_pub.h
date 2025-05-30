// Generated by gencpp from file real_robot_control/force_pub.msg
// DO NOT EDIT!


#ifndef REAL_ROBOT_CONTROL_MESSAGE_FORCE_PUB_H
#define REAL_ROBOT_CONTROL_MESSAGE_FORCE_PUB_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace real_robot_control
{
template <class ContainerAllocator>
struct force_pub_
{
  typedef force_pub_<ContainerAllocator> Type;

  force_pub_()
    : X(0.0)
    , Y(0.0)
    , Z(0.0)
    , MX(0.0)
    , MY(0.0)
    , MZ(0.0)  {
    }
  force_pub_(const ContainerAllocator& _alloc)
    : X(0.0)
    , Y(0.0)
    , Z(0.0)
    , MX(0.0)
    , MY(0.0)
    , MZ(0.0)  {
  (void)_alloc;
    }



   typedef double _X_type;
  _X_type X;

   typedef double _Y_type;
  _Y_type Y;

   typedef double _Z_type;
  _Z_type Z;

   typedef double _MX_type;
  _MX_type MX;

   typedef double _MY_type;
  _MY_type MY;

   typedef double _MZ_type;
  _MZ_type MZ;





  typedef boost::shared_ptr< ::real_robot_control::force_pub_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::real_robot_control::force_pub_<ContainerAllocator> const> ConstPtr;

}; // struct force_pub_

typedef ::real_robot_control::force_pub_<std::allocator<void> > force_pub;

typedef boost::shared_ptr< ::real_robot_control::force_pub > force_pubPtr;
typedef boost::shared_ptr< ::real_robot_control::force_pub const> force_pubConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::real_robot_control::force_pub_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::real_robot_control::force_pub_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::real_robot_control::force_pub_<ContainerAllocator1> & lhs, const ::real_robot_control::force_pub_<ContainerAllocator2> & rhs)
{
  return lhs.X == rhs.X &&
    lhs.Y == rhs.Y &&
    lhs.Z == rhs.Z &&
    lhs.MX == rhs.MX &&
    lhs.MY == rhs.MY &&
    lhs.MZ == rhs.MZ;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::real_robot_control::force_pub_<ContainerAllocator1> & lhs, const ::real_robot_control::force_pub_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace real_robot_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::real_robot_control::force_pub_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::real_robot_control::force_pub_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::real_robot_control::force_pub_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::real_robot_control::force_pub_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::real_robot_control::force_pub_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::real_robot_control::force_pub_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::real_robot_control::force_pub_<ContainerAllocator> >
{
  static const char* value()
  {
    return "01bd0bd3e5758946edad85592cef21e2";
  }

  static const char* value(const ::real_robot_control::force_pub_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x01bd0bd3e5758946ULL;
  static const uint64_t static_value2 = 0xedad85592cef21e2ULL;
};

template<class ContainerAllocator>
struct DataType< ::real_robot_control::force_pub_<ContainerAllocator> >
{
  static const char* value()
  {
    return "real_robot_control/force_pub";
  }

  static const char* value(const ::real_robot_control::force_pub_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::real_robot_control::force_pub_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 X\n"
"float64 Y\n"
"float64 Z\n"
"float64 MX\n"
"float64 MY\n"
"float64 MZ\n"
;
  }

  static const char* value(const ::real_robot_control::force_pub_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::real_robot_control::force_pub_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.X);
      stream.next(m.Y);
      stream.next(m.Z);
      stream.next(m.MX);
      stream.next(m.MY);
      stream.next(m.MZ);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct force_pub_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::real_robot_control::force_pub_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::real_robot_control::force_pub_<ContainerAllocator>& v)
  {
    s << indent << "X: ";
    Printer<double>::stream(s, indent + "  ", v.X);
    s << indent << "Y: ";
    Printer<double>::stream(s, indent + "  ", v.Y);
    s << indent << "Z: ";
    Printer<double>::stream(s, indent + "  ", v.Z);
    s << indent << "MX: ";
    Printer<double>::stream(s, indent + "  ", v.MX);
    s << indent << "MY: ";
    Printer<double>::stream(s, indent + "  ", v.MY);
    s << indent << "MZ: ";
    Printer<double>::stream(s, indent + "  ", v.MZ);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REAL_ROBOT_CONTROL_MESSAGE_FORCE_PUB_H
