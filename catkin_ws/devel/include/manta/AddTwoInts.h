// Generated by gencpp from file manta/AddTwoInts.msg
// DO NOT EDIT!


#ifndef MANTA_MESSAGE_ADDTWOINTS_H
#define MANTA_MESSAGE_ADDTWOINTS_H

#include <ros/service_traits.h>


#include <manta/AddTwoIntsRequest.h>
#include <manta/AddTwoIntsResponse.h>


namespace manta
{

struct AddTwoInts
{

typedef AddTwoIntsRequest Request;
typedef AddTwoIntsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AddTwoInts
} // namespace manta


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::manta::AddTwoInts > {
  static const char* value()
  {
    return "6a2e34150c00229791cc89ff309fff21";
  }

  static const char* value(const ::manta::AddTwoInts&) { return value(); }
};

template<>
struct DataType< ::manta::AddTwoInts > {
  static const char* value()
  {
    return "manta/AddTwoInts";
  }

  static const char* value(const ::manta::AddTwoInts&) { return value(); }
};


// service_traits::MD5Sum< ::manta::AddTwoIntsRequest> should match
// service_traits::MD5Sum< ::manta::AddTwoInts >
template<>
struct MD5Sum< ::manta::AddTwoIntsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::manta::AddTwoInts >::value();
  }
  static const char* value(const ::manta::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::manta::AddTwoIntsRequest> should match
// service_traits::DataType< ::manta::AddTwoInts >
template<>
struct DataType< ::manta::AddTwoIntsRequest>
{
  static const char* value()
  {
    return DataType< ::manta::AddTwoInts >::value();
  }
  static const char* value(const ::manta::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::manta::AddTwoIntsResponse> should match
// service_traits::MD5Sum< ::manta::AddTwoInts >
template<>
struct MD5Sum< ::manta::AddTwoIntsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::manta::AddTwoInts >::value();
  }
  static const char* value(const ::manta::AddTwoIntsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::manta::AddTwoIntsResponse> should match
// service_traits::DataType< ::manta::AddTwoInts >
template<>
struct DataType< ::manta::AddTwoIntsResponse>
{
  static const char* value()
  {
    return DataType< ::manta::AddTwoInts >::value();
  }
  static const char* value(const ::manta::AddTwoIntsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MANTA_MESSAGE_ADDTWOINTS_H
