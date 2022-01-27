// Generated by gencpp from file pkg_task5/msgRosIotResult.msg
// DO NOT EDIT!


#ifndef PKG_TASK5_MESSAGE_MSGROSIOTRESULT_H
#define PKG_TASK5_MESSAGE_MSGROSIOTRESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pkg_task5
{
template <class ContainerAllocator>
struct msgRosIotResult_
{
  typedef msgRosIotResult_<ContainerAllocator> Type;

  msgRosIotResult_()
    : flag_success(false)  {
    }
  msgRosIotResult_(const ContainerAllocator& _alloc)
    : flag_success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _flag_success_type;
  _flag_success_type flag_success;





  typedef boost::shared_ptr< ::pkg_task5::msgRosIotResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pkg_task5::msgRosIotResult_<ContainerAllocator> const> ConstPtr;

}; // struct msgRosIotResult_

typedef ::pkg_task5::msgRosIotResult_<std::allocator<void> > msgRosIotResult;

typedef boost::shared_ptr< ::pkg_task5::msgRosIotResult > msgRosIotResultPtr;
typedef boost::shared_ptr< ::pkg_task5::msgRosIotResult const> msgRosIotResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pkg_task5::msgRosIotResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pkg_task5::msgRosIotResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pkg_task5::msgRosIotResult_<ContainerAllocator1> & lhs, const ::pkg_task5::msgRosIotResult_<ContainerAllocator2> & rhs)
{
  return lhs.flag_success == rhs.flag_success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pkg_task5::msgRosIotResult_<ContainerAllocator1> & lhs, const ::pkg_task5::msgRosIotResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pkg_task5

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::pkg_task5::msgRosIotResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pkg_task5::msgRosIotResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pkg_task5::msgRosIotResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pkg_task5::msgRosIotResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pkg_task5::msgRosIotResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pkg_task5::msgRosIotResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pkg_task5::msgRosIotResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a50265a5223d2c015174ffc1e1099b23";
  }

  static const char* value(const ::pkg_task5::msgRosIotResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa50265a5223d2c01ULL;
  static const uint64_t static_value2 = 0x5174ffc1e1099b23ULL;
};

template<class ContainerAllocator>
struct DataType< ::pkg_task5::msgRosIotResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pkg_task5/msgRosIotResult";
  }

  static const char* value(const ::pkg_task5::msgRosIotResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pkg_task5::msgRosIotResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# result\n"
"bool flag_success\n"
;
  }

  static const char* value(const ::pkg_task5::msgRosIotResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pkg_task5::msgRosIotResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.flag_success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct msgRosIotResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pkg_task5::msgRosIotResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pkg_task5::msgRosIotResult_<ContainerAllocator>& v)
  {
    s << indent << "flag_success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flag_success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PKG_TASK5_MESSAGE_MSGROSIOTRESULT_H
