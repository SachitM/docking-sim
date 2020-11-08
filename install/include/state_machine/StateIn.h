// Generated by gencpp from file state_machine/StateIn.msg
// DO NOT EDIT!


#ifndef STATE_MACHINE_MESSAGE_STATEIN_H
#define STATE_MACHINE_MESSAGE_STATEIN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace state_machine
{
template <class ContainerAllocator>
struct StateIn_
{
  typedef StateIn_<ContainerAllocator> Type;

  StateIn_()
    : header()
    , TransState(0)
    , StateTransitionCond(0)  {
    }
  StateIn_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , TransState(0)
    , StateTransitionCond(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _TransState_type;
  _TransState_type TransState;

   typedef uint8_t _StateTransitionCond_type;
  _StateTransitionCond_type StateTransitionCond;





  typedef boost::shared_ptr< ::state_machine::StateIn_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::state_machine::StateIn_<ContainerAllocator> const> ConstPtr;

}; // struct StateIn_

typedef ::state_machine::StateIn_<std::allocator<void> > StateIn;

typedef boost::shared_ptr< ::state_machine::StateIn > StateInPtr;
typedef boost::shared_ptr< ::state_machine::StateIn const> StateInConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::state_machine::StateIn_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::state_machine::StateIn_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::state_machine::StateIn_<ContainerAllocator1> & lhs, const ::state_machine::StateIn_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.TransState == rhs.TransState &&
    lhs.StateTransitionCond == rhs.StateTransitionCond;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::state_machine::StateIn_<ContainerAllocator1> & lhs, const ::state_machine::StateIn_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace state_machine

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::state_machine::StateIn_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::state_machine::StateIn_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::state_machine::StateIn_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::state_machine::StateIn_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_machine::StateIn_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_machine::StateIn_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::state_machine::StateIn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1c6322d5ada42d15cd42840808483749";
  }

  static const char* value(const ::state_machine::StateIn_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1c6322d5ada42d15ULL;
  static const uint64_t static_value2 = 0xcd42840808483749ULL;
};

template<class ContainerAllocator>
struct DataType< ::state_machine::StateIn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "state_machine/StateIn";
  }

  static const char* value(const ::state_machine::StateIn_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::state_machine::StateIn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header \n"
"uint8 TransState\n"
"uint8 StateTransitionCond\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::state_machine::StateIn_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::state_machine::StateIn_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.TransState);
      stream.next(m.StateTransitionCond);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StateIn_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::state_machine::StateIn_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::state_machine::StateIn_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "TransState: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.TransState);
    s << indent << "StateTransitionCond: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.StateTransitionCond);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STATE_MACHINE_MESSAGE_STATEIN_H
