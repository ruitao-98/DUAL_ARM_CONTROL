// Generated by gencpp from file real_robot_control/pose_pub.msg
// DO NOT EDIT!


#ifndef REAL_ROBOT_CONTROL_MESSAGE_POSE_PUB_H
#define REAL_ROBOT_CONTROL_MESSAGE_POSE_PUB_H


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
struct pose_pub_
{
  typedef pose_pub_<ContainerAllocator> Type;

  pose_pub_()
    : X(0.0)
    , Y(0.0)
    , Z(0.0)
    , RX(0.0)
    , RY(0.0)
    , RZ(0.0)
    , FX(0.0)
    , FY(0.0)
    , FZ(0.0)
    , theta(0.0)  {
    }
  pose_pub_(const ContainerAllocator& _alloc)
    : X(0.0)
    , Y(0.0)
    , Z(0.0)
    , RX(0.0)
    , RY(0.0)
    , RZ(0.0)
    , FX(0.0)
    , FY(0.0)
    , FZ(0.0)
    , theta(0.0)  {
  (void)_alloc;
    }



   typedef double _X_type;
  _X_type X;

   typedef double _Y_type;
  _Y_type Y;

   typedef double _Z_type;
  _Z_type Z;

   typedef double _RX_type;
  _RX_type RX;

   typedef double _RY_type;
  _RY_type RY;

   typedef double _RZ_type;
  _RZ_type RZ;

   typedef double _FX_type;
  _FX_type FX;

   typedef double _FY_type;
  _FY_type FY;

   typedef double _FZ_type;
  _FZ_type FZ;

   typedef double _theta_type;
  _theta_type theta;





  typedef boost::shared_ptr< ::real_robot_control::pose_pub_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::real_robot_control::pose_pub_<ContainerAllocator> const> ConstPtr;

}; // struct pose_pub_

typedef ::real_robot_control::pose_pub_<std::allocator<void> > pose_pub;

typedef boost::shared_ptr< ::real_robot_control::pose_pub > pose_pubPtr;
typedef boost::shared_ptr< ::real_robot_control::pose_pub const> pose_pubConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::real_robot_control::pose_pub_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::real_robot_control::pose_pub_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::real_robot_control::pose_pub_<ContainerAllocator1> & lhs, const ::real_robot_control::pose_pub_<ContainerAllocator2> & rhs)
{
  return lhs.X == rhs.X &&
    lhs.Y == rhs.Y &&
    lhs.Z == rhs.Z &&
    lhs.RX == rhs.RX &&
    lhs.RY == rhs.RY &&
    lhs.RZ == rhs.RZ &&
    lhs.FX == rhs.FX &&
    lhs.FY == rhs.FY &&
    lhs.FZ == rhs.FZ &&
    lhs.theta == rhs.theta;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::real_robot_control::pose_pub_<ContainerAllocator1> & lhs, const ::real_robot_control::pose_pub_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace real_robot_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::real_robot_control::pose_pub_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::real_robot_control::pose_pub_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::real_robot_control::pose_pub_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::real_robot_control::pose_pub_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::real_robot_control::pose_pub_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::real_robot_control::pose_pub_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::real_robot_control::pose_pub_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d20e99fa36280381d3010eacb694aaa0";
  }

  static const char* value(const ::real_robot_control::pose_pub_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd20e99fa36280381ULL;
  static const uint64_t static_value2 = 0xd3010eacb694aaa0ULL;
};

template<class ContainerAllocator>
struct DataType< ::real_robot_control::pose_pub_<ContainerAllocator> >
{
  static const char* value()
  {
    return "real_robot_control/pose_pub";
  }

  static const char* value(const ::real_robot_control::pose_pub_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::real_robot_control::pose_pub_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 X\n"
"float64 Y\n"
"float64 Z\n"
"float64 RX\n"
"float64 RY\n"
"float64 RZ\n"
"float64 FX\n"
"float64 FY\n"
"float64 FZ \n"
"float64 theta\n"
;
  }

  static const char* value(const ::real_robot_control::pose_pub_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::real_robot_control::pose_pub_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.X);
      stream.next(m.Y);
      stream.next(m.Z);
      stream.next(m.RX);
      stream.next(m.RY);
      stream.next(m.RZ);
      stream.next(m.FX);
      stream.next(m.FY);
      stream.next(m.FZ);
      stream.next(m.theta);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct pose_pub_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::real_robot_control::pose_pub_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::real_robot_control::pose_pub_<ContainerAllocator>& v)
  {
    s << indent << "X: ";
    Printer<double>::stream(s, indent + "  ", v.X);
    s << indent << "Y: ";
    Printer<double>::stream(s, indent + "  ", v.Y);
    s << indent << "Z: ";
    Printer<double>::stream(s, indent + "  ", v.Z);
    s << indent << "RX: ";
    Printer<double>::stream(s, indent + "  ", v.RX);
    s << indent << "RY: ";
    Printer<double>::stream(s, indent + "  ", v.RY);
    s << indent << "RZ: ";
    Printer<double>::stream(s, indent + "  ", v.RZ);
    s << indent << "FX: ";
    Printer<double>::stream(s, indent + "  ", v.FX);
    s << indent << "FY: ";
    Printer<double>::stream(s, indent + "  ", v.FY);
    s << indent << "FZ: ";
    Printer<double>::stream(s, indent + "  ", v.FZ);
    s << indent << "theta: ";
    Printer<double>::stream(s, indent + "  ", v.theta);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REAL_ROBOT_CONTROL_MESSAGE_POSE_PUB_H