// Generated by gencpp from file alice_octomap/octomapRequest.msg
// DO NOT EDIT!


#ifndef ALICE_OCTOMAP_MESSAGE_OCTOMAPREQUEST_H
#define ALICE_OCTOMAP_MESSAGE_OCTOMAPREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace alice_octomap
{
template <class ContainerAllocator>
struct octomapRequest_
{
  typedef octomapRequest_<ContainerAllocator> Type;

  octomapRequest_()
    {
    }
  octomapRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::alice_octomap::octomapRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::alice_octomap::octomapRequest_<ContainerAllocator> const> ConstPtr;

}; // struct octomapRequest_

typedef ::alice_octomap::octomapRequest_<std::allocator<void> > octomapRequest;

typedef boost::shared_ptr< ::alice_octomap::octomapRequest > octomapRequestPtr;
typedef boost::shared_ptr< ::alice_octomap::octomapRequest const> octomapRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::alice_octomap::octomapRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::alice_octomap::octomapRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace alice_octomap

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::alice_octomap::octomapRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::alice_octomap::octomapRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::alice_octomap::octomapRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::alice_octomap::octomapRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::alice_octomap::octomapRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::alice_octomap::octomapRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::alice_octomap::octomapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::alice_octomap::octomapRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::alice_octomap::octomapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "alice_octomap/octomapRequest";
  }

  static const char* value(const ::alice_octomap::octomapRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::alice_octomap::octomapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#Service receives no input\n"
"\n"
;
  }

  static const char* value(const ::alice_octomap::octomapRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::alice_octomap::octomapRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct octomapRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::alice_octomap::octomapRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::alice_octomap::octomapRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // ALICE_OCTOMAP_MESSAGE_OCTOMAPREQUEST_H