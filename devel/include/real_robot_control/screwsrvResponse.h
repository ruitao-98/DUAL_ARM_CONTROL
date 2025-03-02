// Generated by gencpp from file real_robot_control/screwsrvResponse.msg
// DO NOT EDIT!


#ifndef REAL_ROBOT_CONTROL_MESSAGE_SCREWSRVRESPONSE_H
#define REAL_ROBOT_CONTROL_MESSAGE_SCREWSRVRESPONSE_H


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
struct screwsrvResponse_
{
  typedef screwsrvResponse_<ContainerAllocator> Type;

  screwsrvResponse_()
    : result(0)  {
    }
  screwsrvResponse_(const ContainerAllocator& _alloc)
    : result(0)  {
  (void)_alloc;
    }



   typedef int32_t _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::real_robot_control::screwsrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::real_robot_control::screwsrvResponse_<ContainerAllocator> const> ConstPtr;

}; // struct screwsrvResponse_

typedef ::real_robot_control::screwsrvResponse_<std::allocator<void> > screwsrvResponse;

typedef boost::shared_ptr< ::real_robot_control::screwsrvResponse > screwsrvResponsePtr;
typedef boost::shared_ptr< ::real_robot_control::screwsrvResponse const> screwsrvResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::real_robot_control::screwsrvResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::real_robot_control::screwsrvResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::real_robot_control::screwsrvResponse_<ContainerAllocator1> & lhs, const ::real_robot_control::screwsrvResponse_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::real_robot_control::screwsrvResponse_<ContainerAllocator1> & lhs, const ::real_robot_control::screwsrvResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace real_robot_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::real_robot_control::screwsrvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::real_robot_control::screwsrvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::real_robot_control::screwsrvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::real_robot_control::screwsrvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::real_robot_control::screwsrvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::real_robot_control::screwsrvResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::real_robot_control::screwsrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const ::real_robot_control::screwsrvResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::real_robot_control::screwsrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "real_robot_control/screwsrvResponse";
  }

  static const char* value(const ::real_robot_control::screwsrvResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::real_robot_control::screwsrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#最终结果\n"
"int32 result\n"
"\n"
;
  }

  static const char* value(const ::real_robot_control::screwsrvResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::real_robot_control::screwsrvResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct screwsrvResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::real_robot_control::screwsrvResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::real_robot_control::screwsrvResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<int32_t>::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REAL_ROBOT_CONTROL_MESSAGE_SCREWSRVRESPONSE_H
