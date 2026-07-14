# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vision_msgs:msg/TargetCoord.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TargetCoord(type):
    """Metaclass of message 'TargetCoord'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('vision_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vision_msgs.msg.TargetCoord')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__target_coord
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__target_coord
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__target_coord
            cls._TYPE_SUPPORT = module.type_support_msg__msg__target_coord
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__target_coord

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TargetCoord(metaclass=Metaclass_TargetCoord):
    """Message class 'TargetCoord'."""

    __slots__ = [
        '_p_x',
        '_p_y',
        '_conf',
        '_class_id',
    ]

    _fields_and_field_types = {
        'p_x': 'float',
        'p_y': 'float',
        'conf': 'float',
        'class_id': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.p_x = kwargs.get('p_x', float())
        self.p_y = kwargs.get('p_y', float())
        self.conf = kwargs.get('conf', float())
        self.class_id = kwargs.get('class_id', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.p_x != other.p_x:
            return False
        if self.p_y != other.p_y:
            return False
        if self.conf != other.conf:
            return False
        if self.class_id != other.class_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def p_x(self):
        """Message field 'p_x'."""
        return self._p_x

    @p_x.setter
    def p_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'p_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'p_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._p_x = value

    @builtins.property
    def p_y(self):
        """Message field 'p_y'."""
        return self._p_y

    @p_y.setter
    def p_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'p_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'p_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._p_y = value

    @builtins.property
    def conf(self):
        """Message field 'conf'."""
        return self._conf

    @conf.setter
    def conf(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'conf' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'conf' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._conf = value

    @builtins.property
    def class_id(self):
        """Message field 'class_id'."""
        return self._class_id

    @class_id.setter
    def class_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'class_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'class_id' field must be an unsigned integer in [0, 255]"
        self._class_id = value
