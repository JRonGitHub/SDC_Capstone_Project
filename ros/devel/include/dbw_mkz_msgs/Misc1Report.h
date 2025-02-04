// Generated by gencpp from file dbw_mkz_msgs/Misc1Report.msg
// DO NOT EDIT!


#ifndef DBW_MKZ_MSGS_MESSAGE_MISC1REPORT_H
#define DBW_MKZ_MSGS_MESSAGE_MISC1REPORT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <dbw_mkz_msgs/TurnSignal.h>
#include <dbw_mkz_msgs/Wiper.h>
#include <dbw_mkz_msgs/AmbientLight.h>

namespace dbw_mkz_msgs
{
template <class ContainerAllocator>
struct Misc1Report_
{
  typedef Misc1Report_<ContainerAllocator> Type;

  Misc1Report_()
    : header()
    , turn_signal()
    , high_beam_headlights(false)
    , wiper()
    , ambient_light()
    , btn_cc_on(false)
    , btn_cc_off(false)
    , btn_cc_on_off(false)
    , btn_cc_res(false)
    , btn_cc_cncl(false)
    , btn_cc_res_cncl(false)
    , btn_cc_res_inc(false)
    , btn_cc_res_dec(false)
    , btn_cc_set_inc(false)
    , btn_cc_set_dec(false)
    , btn_cc_gap_inc(false)
    , btn_cc_gap_dec(false)
    , btn_la_on_off(false)
    , btn_ld_ok(false)
    , btn_ld_up(false)
    , btn_ld_down(false)
    , btn_ld_left(false)
    , btn_ld_right(false)
    , fault_bus(false)
    , door_driver(false)
    , door_passenger(false)
    , door_rear_left(false)
    , door_rear_right(false)
    , door_hood(false)
    , door_trunk(false)
    , passenger_detect(false)
    , passenger_airbag(false)
    , buckle_driver(false)
    , buckle_passenger(false)  {
    }
  Misc1Report_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , turn_signal(_alloc)
    , high_beam_headlights(false)
    , wiper(_alloc)
    , ambient_light(_alloc)
    , btn_cc_on(false)
    , btn_cc_off(false)
    , btn_cc_on_off(false)
    , btn_cc_res(false)
    , btn_cc_cncl(false)
    , btn_cc_res_cncl(false)
    , btn_cc_res_inc(false)
    , btn_cc_res_dec(false)
    , btn_cc_set_inc(false)
    , btn_cc_set_dec(false)
    , btn_cc_gap_inc(false)
    , btn_cc_gap_dec(false)
    , btn_la_on_off(false)
    , btn_ld_ok(false)
    , btn_ld_up(false)
    , btn_ld_down(false)
    , btn_ld_left(false)
    , btn_ld_right(false)
    , fault_bus(false)
    , door_driver(false)
    , door_passenger(false)
    , door_rear_left(false)
    , door_rear_right(false)
    , door_hood(false)
    , door_trunk(false)
    , passenger_detect(false)
    , passenger_airbag(false)
    , buckle_driver(false)
    , buckle_passenger(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::dbw_mkz_msgs::TurnSignal_<ContainerAllocator>  _turn_signal_type;
  _turn_signal_type turn_signal;

   typedef uint8_t _high_beam_headlights_type;
  _high_beam_headlights_type high_beam_headlights;

   typedef  ::dbw_mkz_msgs::Wiper_<ContainerAllocator>  _wiper_type;
  _wiper_type wiper;

   typedef  ::dbw_mkz_msgs::AmbientLight_<ContainerAllocator>  _ambient_light_type;
  _ambient_light_type ambient_light;

   typedef uint8_t _btn_cc_on_type;
  _btn_cc_on_type btn_cc_on;

   typedef uint8_t _btn_cc_off_type;
  _btn_cc_off_type btn_cc_off;

   typedef uint8_t _btn_cc_on_off_type;
  _btn_cc_on_off_type btn_cc_on_off;

   typedef uint8_t _btn_cc_res_type;
  _btn_cc_res_type btn_cc_res;

   typedef uint8_t _btn_cc_cncl_type;
  _btn_cc_cncl_type btn_cc_cncl;

   typedef uint8_t _btn_cc_res_cncl_type;
  _btn_cc_res_cncl_type btn_cc_res_cncl;

   typedef uint8_t _btn_cc_res_inc_type;
  _btn_cc_res_inc_type btn_cc_res_inc;

   typedef uint8_t _btn_cc_res_dec_type;
  _btn_cc_res_dec_type btn_cc_res_dec;

   typedef uint8_t _btn_cc_set_inc_type;
  _btn_cc_set_inc_type btn_cc_set_inc;

   typedef uint8_t _btn_cc_set_dec_type;
  _btn_cc_set_dec_type btn_cc_set_dec;

   typedef uint8_t _btn_cc_gap_inc_type;
  _btn_cc_gap_inc_type btn_cc_gap_inc;

   typedef uint8_t _btn_cc_gap_dec_type;
  _btn_cc_gap_dec_type btn_cc_gap_dec;

   typedef uint8_t _btn_la_on_off_type;
  _btn_la_on_off_type btn_la_on_off;

   typedef uint8_t _btn_ld_ok_type;
  _btn_ld_ok_type btn_ld_ok;

   typedef uint8_t _btn_ld_up_type;
  _btn_ld_up_type btn_ld_up;

   typedef uint8_t _btn_ld_down_type;
  _btn_ld_down_type btn_ld_down;

   typedef uint8_t _btn_ld_left_type;
  _btn_ld_left_type btn_ld_left;

   typedef uint8_t _btn_ld_right_type;
  _btn_ld_right_type btn_ld_right;

   typedef uint8_t _fault_bus_type;
  _fault_bus_type fault_bus;

   typedef uint8_t _door_driver_type;
  _door_driver_type door_driver;

   typedef uint8_t _door_passenger_type;
  _door_passenger_type door_passenger;

   typedef uint8_t _door_rear_left_type;
  _door_rear_left_type door_rear_left;

   typedef uint8_t _door_rear_right_type;
  _door_rear_right_type door_rear_right;

   typedef uint8_t _door_hood_type;
  _door_hood_type door_hood;

   typedef uint8_t _door_trunk_type;
  _door_trunk_type door_trunk;

   typedef uint8_t _passenger_detect_type;
  _passenger_detect_type passenger_detect;

   typedef uint8_t _passenger_airbag_type;
  _passenger_airbag_type passenger_airbag;

   typedef uint8_t _buckle_driver_type;
  _buckle_driver_type buckle_driver;

   typedef uint8_t _buckle_passenger_type;
  _buckle_passenger_type buckle_passenger;





  typedef boost::shared_ptr< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> const> ConstPtr;

}; // struct Misc1Report_

typedef ::dbw_mkz_msgs::Misc1Report_<std::allocator<void> > Misc1Report;

typedef boost::shared_ptr< ::dbw_mkz_msgs::Misc1Report > Misc1ReportPtr;
typedef boost::shared_ptr< ::dbw_mkz_msgs::Misc1Report const> Misc1ReportConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dbw_mkz_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'dbw_mkz_msgs': ['/home/guilins/catkin_ws/src/SDC_Capstone_Project/ros/src/dbw_mkz_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5e430a9996eeb7a76d2abf0e360213d3";
  }

  static const char* value(const ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5e430a9996eeb7a7ULL;
  static const uint64_t static_value2 = 0x6d2abf0e360213d3ULL;
};

template<class ContainerAllocator>
struct DataType< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dbw_mkz_msgs/Misc1Report";
  }

  static const char* value(const ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
# Turn Signal enumeration\n\
TurnSignal turn_signal\n\
\n\
# High beams\n\
bool high_beam_headlights\n\
\n\
# Front Windshield Wipers enumeration\n\
Wiper wiper\n\
\n\
# Ambient Light Sensor enumeration\n\
AmbientLight ambient_light\n\
\n\
# Buttons\n\
bool btn_cc_on        # Cruise Control On\n\
bool btn_cc_off       # Cruise Control Off\n\
bool btn_cc_on_off    # Cruise Control On/Off Toggle\n\
bool btn_cc_res       # Cruise Control Resume\n\
bool btn_cc_cncl      # Cruise Control Cancel\n\
bool btn_cc_res_cncl  # Cruise Control Resume/Cancel\n\
bool btn_cc_res_inc   # Cruise Control Resume+\n\
bool btn_cc_res_dec   # Cruise Control Resume-\n\
bool btn_cc_set_inc   # Cruise Control Set+\n\
bool btn_cc_set_dec   # Cruise Control Set-\n\
bool btn_cc_gap_inc   # Cruise Control Gap+\n\
bool btn_cc_gap_dec   # Cruise Control Gap-\n\
bool btn_la_on_off    # Lane Assist On/Off Toggle\n\
bool btn_ld_ok        # Left D-Pad OK\n\
bool btn_ld_up        # Left D-Pad Up\n\
bool btn_ld_down      # Left D-Pad Down\n\
bool btn_ld_left      # Left D-Pad Left\n\
bool btn_ld_right     # Left D-Pad Right\n\
\n\
# Faults\n\
bool fault_bus\n\
\n\
# Doors\n\
bool door_driver\n\
bool door_passenger\n\
bool door_rear_left\n\
bool door_rear_right\n\
bool door_hood\n\
bool door_trunk\n\
\n\
# Passenger seat\n\
bool passenger_detect\n\
bool passenger_airbag\n\
\n\
# Seat Belts\n\
bool buckle_driver\n\
bool buckle_passenger\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: dbw_mkz_msgs/TurnSignal\n\
uint8 value\n\
\n\
uint8 NONE=0\n\
uint8 LEFT=1\n\
uint8 RIGHT=2\n\
\n\
================================================================================\n\
MSG: dbw_mkz_msgs/Wiper\n\
uint8 status\n\
\n\
uint8 OFF=0\n\
uint8 AUTO_OFF=1\n\
uint8 OFF_MOVING=2\n\
uint8 MANUAL_OFF=3\n\
uint8 MANUAL_ON=4\n\
uint8 MANUAL_LOW=5\n\
uint8 MANUAL_HIGH=6\n\
uint8 MIST_FLICK=7\n\
uint8 WASH=8\n\
uint8 AUTO_LOW=9\n\
uint8 AUTO_HIGH=10\n\
uint8 COURTESYWIPE=11\n\
uint8 AUTO_ADJUST=12\n\
uint8 RESERVED=13\n\
uint8 STALLED=14\n\
uint8 NO_DATA=15\n\
\n\
================================================================================\n\
MSG: dbw_mkz_msgs/AmbientLight\n\
uint8 status\n\
\n\
uint8 DARK=0\n\
uint8 LIGHT=1\n\
uint8 TWILIGHT=2\n\
uint8 TUNNEL_ON=3\n\
uint8 TUNNEL_OFF=4\n\
uint8 NO_DATA=7\n\
";
  }

  static const char* value(const ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.turn_signal);
      stream.next(m.high_beam_headlights);
      stream.next(m.wiper);
      stream.next(m.ambient_light);
      stream.next(m.btn_cc_on);
      stream.next(m.btn_cc_off);
      stream.next(m.btn_cc_on_off);
      stream.next(m.btn_cc_res);
      stream.next(m.btn_cc_cncl);
      stream.next(m.btn_cc_res_cncl);
      stream.next(m.btn_cc_res_inc);
      stream.next(m.btn_cc_res_dec);
      stream.next(m.btn_cc_set_inc);
      stream.next(m.btn_cc_set_dec);
      stream.next(m.btn_cc_gap_inc);
      stream.next(m.btn_cc_gap_dec);
      stream.next(m.btn_la_on_off);
      stream.next(m.btn_ld_ok);
      stream.next(m.btn_ld_up);
      stream.next(m.btn_ld_down);
      stream.next(m.btn_ld_left);
      stream.next(m.btn_ld_right);
      stream.next(m.fault_bus);
      stream.next(m.door_driver);
      stream.next(m.door_passenger);
      stream.next(m.door_rear_left);
      stream.next(m.door_rear_right);
      stream.next(m.door_hood);
      stream.next(m.door_trunk);
      stream.next(m.passenger_detect);
      stream.next(m.passenger_airbag);
      stream.next(m.buckle_driver);
      stream.next(m.buckle_passenger);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Misc1Report_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dbw_mkz_msgs::Misc1Report_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "turn_signal: ";
    s << std::endl;
    Printer< ::dbw_mkz_msgs::TurnSignal_<ContainerAllocator> >::stream(s, indent + "  ", v.turn_signal);
    s << indent << "high_beam_headlights: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.high_beam_headlights);
    s << indent << "wiper: ";
    s << std::endl;
    Printer< ::dbw_mkz_msgs::Wiper_<ContainerAllocator> >::stream(s, indent + "  ", v.wiper);
    s << indent << "ambient_light: ";
    s << std::endl;
    Printer< ::dbw_mkz_msgs::AmbientLight_<ContainerAllocator> >::stream(s, indent + "  ", v.ambient_light);
    s << indent << "btn_cc_on: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_on);
    s << indent << "btn_cc_off: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_off);
    s << indent << "btn_cc_on_off: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_on_off);
    s << indent << "btn_cc_res: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_res);
    s << indent << "btn_cc_cncl: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_cncl);
    s << indent << "btn_cc_res_cncl: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_res_cncl);
    s << indent << "btn_cc_res_inc: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_res_inc);
    s << indent << "btn_cc_res_dec: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_res_dec);
    s << indent << "btn_cc_set_inc: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_set_inc);
    s << indent << "btn_cc_set_dec: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_set_dec);
    s << indent << "btn_cc_gap_inc: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_gap_inc);
    s << indent << "btn_cc_gap_dec: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_cc_gap_dec);
    s << indent << "btn_la_on_off: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_la_on_off);
    s << indent << "btn_ld_ok: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_ld_ok);
    s << indent << "btn_ld_up: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_ld_up);
    s << indent << "btn_ld_down: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_ld_down);
    s << indent << "btn_ld_left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_ld_left);
    s << indent << "btn_ld_right: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.btn_ld_right);
    s << indent << "fault_bus: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.fault_bus);
    s << indent << "door_driver: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.door_driver);
    s << indent << "door_passenger: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.door_passenger);
    s << indent << "door_rear_left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.door_rear_left);
    s << indent << "door_rear_right: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.door_rear_right);
    s << indent << "door_hood: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.door_hood);
    s << indent << "door_trunk: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.door_trunk);
    s << indent << "passenger_detect: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.passenger_detect);
    s << indent << "passenger_airbag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.passenger_airbag);
    s << indent << "buckle_driver: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.buckle_driver);
    s << indent << "buckle_passenger: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.buckle_passenger);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DBW_MKZ_MSGS_MESSAGE_MISC1REPORT_H
