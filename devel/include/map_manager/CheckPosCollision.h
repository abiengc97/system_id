// Generated by gencpp from file map_manager/CheckPosCollision.msg
// DO NOT EDIT!


#ifndef MAP_MANAGER_MESSAGE_CHECKPOSCOLLISION_H
#define MAP_MANAGER_MESSAGE_CHECKPOSCOLLISION_H

#include <ros/service_traits.h>


#include <map_manager/CheckPosCollisionRequest.h>
#include <map_manager/CheckPosCollisionResponse.h>


namespace map_manager
{

struct CheckPosCollision
{

typedef CheckPosCollisionRequest Request;
typedef CheckPosCollisionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct CheckPosCollision
} // namespace map_manager


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::map_manager::CheckPosCollision > {
  static const char* value()
  {
    return "5b2617caaa5faa9b1c910b8c4d0cf2ea";
  }

  static const char* value(const ::map_manager::CheckPosCollision&) { return value(); }
};

template<>
struct DataType< ::map_manager::CheckPosCollision > {
  static const char* value()
  {
    return "map_manager/CheckPosCollision";
  }

  static const char* value(const ::map_manager::CheckPosCollision&) { return value(); }
};


// service_traits::MD5Sum< ::map_manager::CheckPosCollisionRequest> should match
// service_traits::MD5Sum< ::map_manager::CheckPosCollision >
template<>
struct MD5Sum< ::map_manager::CheckPosCollisionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::map_manager::CheckPosCollision >::value();
  }
  static const char* value(const ::map_manager::CheckPosCollisionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::map_manager::CheckPosCollisionRequest> should match
// service_traits::DataType< ::map_manager::CheckPosCollision >
template<>
struct DataType< ::map_manager::CheckPosCollisionRequest>
{
  static const char* value()
  {
    return DataType< ::map_manager::CheckPosCollision >::value();
  }
  static const char* value(const ::map_manager::CheckPosCollisionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::map_manager::CheckPosCollisionResponse> should match
// service_traits::MD5Sum< ::map_manager::CheckPosCollision >
template<>
struct MD5Sum< ::map_manager::CheckPosCollisionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::map_manager::CheckPosCollision >::value();
  }
  static const char* value(const ::map_manager::CheckPosCollisionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::map_manager::CheckPosCollisionResponse> should match
// service_traits::DataType< ::map_manager::CheckPosCollision >
template<>
struct DataType< ::map_manager::CheckPosCollisionResponse>
{
  static const char* value()
  {
    return DataType< ::map_manager::CheckPosCollision >::value();
  }
  static const char* value(const ::map_manager::CheckPosCollisionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MAP_MANAGER_MESSAGE_CHECKPOSCOLLISION_H
