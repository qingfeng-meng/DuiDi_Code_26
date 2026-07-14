// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/TargetCoord.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__TARGET_COORD__STRUCT_HPP_
#define VISION_MSGS__MSG__DETAIL__TARGET_COORD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__TargetCoord __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__TargetCoord __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TargetCoord_
{
  using Type = TargetCoord_<ContainerAllocator>;

  explicit TargetCoord_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->p_x = 0.0f;
      this->p_y = 0.0f;
      this->conf = 0.0f;
      this->class_id = 0;
    }
  }

  explicit TargetCoord_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->p_x = 0.0f;
      this->p_y = 0.0f;
      this->conf = 0.0f;
      this->class_id = 0;
    }
  }

  // field types and members
  using _p_x_type =
    float;
  _p_x_type p_x;
  using _p_y_type =
    float;
  _p_y_type p_y;
  using _conf_type =
    float;
  _conf_type conf;
  using _class_id_type =
    uint8_t;
  _class_id_type class_id;

  // setters for named parameter idiom
  Type & set__p_x(
    const float & _arg)
  {
    this->p_x = _arg;
    return *this;
  }
  Type & set__p_y(
    const float & _arg)
  {
    this->p_y = _arg;
    return *this;
  }
  Type & set__conf(
    const float & _arg)
  {
    this->conf = _arg;
    return *this;
  }
  Type & set__class_id(
    const uint8_t & _arg)
  {
    this->class_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::TargetCoord_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::TargetCoord_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::TargetCoord_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::TargetCoord_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::TargetCoord_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::TargetCoord_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::TargetCoord_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::TargetCoord_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::TargetCoord_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::TargetCoord_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__TargetCoord
    std::shared_ptr<vision_msgs::msg::TargetCoord_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__TargetCoord
    std::shared_ptr<vision_msgs::msg::TargetCoord_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetCoord_ & other) const
  {
    if (this->p_x != other.p_x) {
      return false;
    }
    if (this->p_y != other.p_y) {
      return false;
    }
    if (this->conf != other.conf) {
      return false;
    }
    if (this->class_id != other.class_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetCoord_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetCoord_

// alias to use template instance with default allocator
using TargetCoord =
  vision_msgs::msg::TargetCoord_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // VISION_MSGS__MSG__DETAIL__TARGET_COORD__STRUCT_HPP_
