// Generated by gencpp from file real_robot_control/leftrobotsrv.msg
// DO NOT EDIT!


#ifndef REAL_ROBOT_CONTROL_MESSAGE_LEFTROBOTSRV_H
#define REAL_ROBOT_CONTROL_MESSAGE_LEFTROBOTSRV_H

#include <ros/service_traits.h>


#include <real_robot_control/leftrobotsrvRequest.h>
#include <real_robot_control/leftrobotsrvResponse.h>


namespace real_robot_control
{

struct leftrobotsrv
{

typedef leftrobotsrvRequest Request;
typedef leftrobotsrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct leftrobotsrv
} // namespace real_robot_control


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::real_robot_control::leftrobotsrv > {
  static const char* value()
  {
    return "7b18929c222448a67fda0cd87f44304d";
  }

  static const char* value(const ::real_robot_control::leftrobotsrv&) { return value(); }
};

template<>
struct DataType< ::real_robot_control::leftrobotsrv > {
  static const char* value()
  {
    return "real_robot_control/leftrobotsrv";
  }

  static const char* value(const ::real_robot_control::leftrobotsrv&) { return value(); }
};


// service_traits::MD5Sum< ::real_robot_control::leftrobotsrvRequest> should match
// service_traits::MD5Sum< ::real_robot_control::leftrobotsrv >
template<>
struct MD5Sum< ::real_robot_control::leftrobotsrvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::real_robot_control::leftrobotsrv >::value();
  }
  static const char* value(const ::real_robot_control::leftrobotsrvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::real_robot_control::leftrobotsrvRequest> should match
// service_traits::DataType< ::real_robot_control::leftrobotsrv >
template<>
struct DataType< ::real_robot_control::leftrobotsrvRequest>
{
  static const char* value()
  {
    return DataType< ::real_robot_control::leftrobotsrv >::value();
  }
  static const char* value(const ::real_robot_control::leftrobotsrvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::real_robot_control::leftrobotsrvResponse> should match
// service_traits::MD5Sum< ::real_robot_control::leftrobotsrv >
template<>
struct MD5Sum< ::real_robot_control::leftrobotsrvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::real_robot_control::leftrobotsrv >::value();
  }
  static const char* value(const ::real_robot_control::leftrobotsrvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::real_robot_control::leftrobotsrvResponse> should match
// service_traits::DataType< ::real_robot_control::leftrobotsrv >
template<>
struct DataType< ::real_robot_control::leftrobotsrvResponse>
{
  static const char* value()
  {
    return DataType< ::real_robot_control::leftrobotsrv >::value();
  }
  static const char* value(const ::real_robot_control::leftrobotsrvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // REAL_ROBOT_CONTROL_MESSAGE_LEFTROBOTSRV_H
