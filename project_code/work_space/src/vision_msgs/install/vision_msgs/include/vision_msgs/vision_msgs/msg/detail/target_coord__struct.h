// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vision_msgs:msg/TargetCoord.idl
// generated code does not contain a copyright notice

#ifndef VISION_MSGS__MSG__DETAIL__TARGET_COORD__STRUCT_H_
#define VISION_MSGS__MSG__DETAIL__TARGET_COORD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/TargetCoord in the package vision_msgs.
/**
  * 坐标
 */
typedef struct vision_msgs__msg__TargetCoord
{
  float p_x;
  float p_y;
  /// 置信度
  float conf;
  /// 目标类别
  uint8_t class_id;
} vision_msgs__msg__TargetCoord;

// Struct for a sequence of vision_msgs__msg__TargetCoord.
typedef struct vision_msgs__msg__TargetCoord__Sequence
{
  vision_msgs__msg__TargetCoord * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vision_msgs__msg__TargetCoord__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VISION_MSGS__MSG__DETAIL__TARGET_COORD__STRUCT_H_
