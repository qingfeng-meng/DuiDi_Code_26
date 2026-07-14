// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vision_msgs:msg/TargetCoord.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__TARGET_COORD__BUILDER_HPP_
#define VISION_MSGS__MSG__DETAIL__TARGET_COORD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vision_msgs/msg/detail/target_coord__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vision_msgs
{

namespace msg
{

namespace builder
{

class Init_TargetCoord_class_id
{
public:
  explicit Init_TargetCoord_class_id(::vision_msgs::msg::TargetCoord & msg)
  : msg_(msg)
  {}
  ::vision_msgs::msg::TargetCoord class_id(::vision_msgs::msg::TargetCoord::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vision_msgs::msg::TargetCoord msg_;
};

class Init_TargetCoord_conf
{
public:
  explicit Init_TargetCoord_conf(::vision_msgs::msg::TargetCoord & msg)
  : msg_(msg)
  {}
  Init_TargetCoord_class_id conf(::vision_msgs::msg::TargetCoord::_conf_type arg)
  {
    msg_.conf = std::move(arg);
    return Init_TargetCoord_class_id(msg_);
  }

private:
  ::vision_msgs::msg::TargetCoord msg_;
};

class Init_TargetCoord_p_y
{
public:
  explicit Init_TargetCoord_p_y(::vision_msgs::msg::TargetCoord & msg)
  : msg_(msg)
  {}
  Init_TargetCoord_conf p_y(::vision_msgs::msg::TargetCoord::_p_y_type arg)
  {
    msg_.p_y = std::move(arg);
    return Init_TargetCoord_conf(msg_);
  }

private:
  ::vision_msgs::msg::TargetCoord msg_;
};

class Init_TargetCoord_p_x
{
public:
  Init_TargetCoord_p_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetCoord_p_y p_x(::vision_msgs::msg::TargetCoord::_p_x_type arg)
  {
    msg_.p_x = std::move(arg);
    return Init_TargetCoord_p_y(msg_);
  }

private:
  ::vision_msgs::msg::TargetCoord msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vision_msgs::msg::TargetCoord>()
{
  return vision_msgs::msg::builder::Init_TargetCoord_p_x();
}

}  // namespace vision_msgs

#endif  // VISION_MSGS__MSG__DETAIL__TARGET_COORD__BUILDER_HPP_
