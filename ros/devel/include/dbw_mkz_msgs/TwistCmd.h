// Generated by gencpp from file dbw_mkz_msgs/TwistCmd.msg
// DO NOT EDIT!


#ifndef DBW_MKZ_MSGS_MESSAGE_TWISTCMD_H
#define DBW_MKZ_MSGS_MESSAGE_TWISTCMD_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Twist.h>

namespace dbw_mkz_msgs
{
template <class ContainerAllocator>
struct TwistCmd_
{
  typedef TwistCmd_<ContainerAllocator> Type;

  TwistCmd_()
    : twist()
    , accel_limit(0.0)
    , decel_limit(0.0)  {
    }
  TwistCmd_(const ContainerAllocator& _alloc)
    : twist(_alloc)
    , accel_limit(0.0)
    , decel_limit(0.0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef float _accel_limit_type;
  _accel_limit_type accel_limit;

   typedef float _decel_limit_type;
  _decel_limit_type decel_limit;





  typedef boost::shared_ptr< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> const> ConstPtr;

}; // struct TwistCmd_

typedef ::dbw_mkz_msgs::TwistCmd_<std::allocator<void> > TwistCmd;

typedef boost::shared_ptr< ::dbw_mkz_msgs::TwistCmd > TwistCmdPtr;
typedef boost::shared_ptr< ::dbw_mkz_msgs::TwistCmd const> TwistCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dbw_mkz_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'dbw_mkz_msgs': ['/home/guilins/catkin_ws/src/SDC_Capstone_Project/ros/src/dbw_mkz_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ef873397d04f1a8acdfa4bcab4392286";
  }

  static const char* value(const ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xef873397d04f1a8aULL;
  static const uint64_t static_value2 = 0xcdfa4bcab4392286ULL;
};

template<class ContainerAllocator>
struct DataType< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dbw_mkz_msgs/TwistCmd";
  }

  static const char* value(const ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Twist twist\n\
float32 accel_limit # m/s^2, zero = no limit\n\
float32 decel_limit # m/s^2, zero = no limit\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.twist);
      stream.next(m.accel_limit);
      stream.next(m.decel_limit);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TwistCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dbw_mkz_msgs::TwistCmd_<ContainerAllocator>& v)
  {
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
    s << indent << "accel_limit: ";
    Printer<float>::stream(s, indent + "  ", v.accel_limit);
    s << indent << "decel_limit: ";
    Printer<float>::stream(s, indent + "  ", v.decel_limit);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DBW_MKZ_MSGS_MESSAGE_TWISTCMD_H
