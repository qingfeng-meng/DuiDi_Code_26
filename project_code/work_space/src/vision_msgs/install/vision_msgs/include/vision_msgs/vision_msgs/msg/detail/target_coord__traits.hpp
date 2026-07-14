// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vision_msgs:msg/TargetCoord.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__TARGET_COORD__TRAITS_HPP_
#define VISION_MSGS__MSG__DETAIL__TARGET_COORD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vision_msgs/msg/detail/target_coord__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vision_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TargetCoord & msg,
  std::ostream & out)
{
  out << "{";
  // member: p_x
  {
    out << "p_x: ";
    rosidl_generator_traits::value_to_yaml(msg.p_x, out);
    out << ", ";
  }

  // member: p_y
  {
    out << "p_y: ";
    rosidl_generator_traits::value_to_yaml(msg.p_y, out);
    out << ", ";
  }

  // member: conf
  {
    out << "conf: ";
    rosidl_generator_traits::value_to_yaml(msg.conf, out);
    out << ", ";
  }

  // member: class_id
  {
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TargetCoord & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: p_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "p_x: ";
    rosidl_generator_traits::value_to_yaml(msg.p_x, out);
    out << "\n";
  }

  // member: p_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "p_y: ";
    rosidl_generator_traits::value_to_yaml(msg.p_y, out);
    out << "\n";
  }

  // member: conf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "conf: ";
    rosidl_generator_traits::value_to_yaml(msg.conf, out);
    out << "\n";
  }

  // member: class_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TargetCoord & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace vision_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vision_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vision_msgs::msg::TargetCoord & msg,
  std::ostream & out, size_t indentation = 0)
{
  vision_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vision_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vision_msgs::msg::TargetCoord & msg)
{
  return vision_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vision_msgs::msg::TargetCoord>()
{
  return "vision_msgs::msg::TargetCoord";
}

template<>
inline const char * name<vision_msgs::msg::TargetCoord>()
{
  return "vision_msgs/msg/TargetCoord";
}

template<>
struct has_fixed_size<vision_msgs::msg::TargetCoord>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vision_msgs::msg::TargetCoord>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vision_msgs::msg::TargetCoord>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VISION_MSGS__MSG__DETAIL__TARGET_COORD__TRAITS_HPP_
