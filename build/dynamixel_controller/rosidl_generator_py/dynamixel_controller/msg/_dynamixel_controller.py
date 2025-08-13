# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dynamixel_controller:msg/DynamixelController.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DynamixelController(type):
    """Metaclass of message 'DynamixelController'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'PING': 1,
        'READ_DATA': 2,
        'WRITE_DATA': 3,
        'REG_WRITE': 4,
        'ACTION': 5,
        'FACTORY_RESET': 6,
        'REBOOT': 8,
        'SYNC_READ': 130,
        'SYNC_WRITE': 131,
        'BULK_READ': 146,
        'BULK_WRITE': 147,
        'MODEL_NUMBER': 0,
        'MODEL_INFORMATION': 2,
        'FIRMWARE_VERSION': 6,
        'ID': 7,
        'BAUD_RATE': 8,
        'RETURN_DELAY_TIME': 9,
        'DRIVE_MODE': 10,
        'OPERATING_MODE': 11,
        'SECONDARY_ID': 12,
        'PROTOCOL_TYPE': 13,
        'HOMING_OFFSET': 20,
        'MOVING_THRESHOLD': 24,
        'TEMPERATURE_LIMIT': 31,
        'MAX_VOLTAGE_LIMIT': 32,
        'MIN_VOLTAGE_LIMIT': 34,
        'PWM_LIMIT': 36,
        'CURRENT_LIMIT': 38,
        'VELOCITY_LIMIT': 40,
        'MAX_POSITION_LIMIT': 44,
        'MIN_POSITION_LIMIT': 48,
        'TORQUE_ENABLE': 64,
        'LED': 65,
        'STATUS_RETURN_LEVEL': 68,
        'REGISTERED_INSTRUCTION': 69,
        'HARDWARE_ERROR_STATUS': 70,
        'VELOCITY_I_GAIN': 76,
        'VELOCITY_P_GAIN': 78,
        'POSITION_D_GAIN': 80,
        'POSITION_I_GAIN': 82,
        'POSITION_P_GAIN': 84,
        'FEEDFORWARD_2_GAIN': 88,
        'FEEDFORWARD_1_GAIN': 90,
        'BUS_WATCHDOG': 98,
        'GOAL_PWM': 100,
        'GOAL_CURRENT': 102,
        'GOAL_VELOCITY': 104,
        'PROFILE_ACCELERATION': 108,
        'PROFILE_VELOCITY': 112,
        'GOAL_POSITION': 116,
        'REALTIME_TICK': 120,
        'MOVING': 122,
        'PRESENT_PWM': 124,
        'PRESENT_CURRENT': 126,
        'PRESENT_VELOCITY': 128,
        'PRESENT_POSITION': 132,
        'PRESENT_INPUT_VOLTAGE': 144,
        'PRESENT_TEMPERATURE': 146,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dynamixel_controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dynamixel_controller.msg.DynamixelController')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__dynamixel_controller
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__dynamixel_controller
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__dynamixel_controller
            cls._TYPE_SUPPORT = module.type_support_msg__msg__dynamixel_controller
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__dynamixel_controller

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'PING': cls.__constants['PING'],
            'READ_DATA': cls.__constants['READ_DATA'],
            'WRITE_DATA': cls.__constants['WRITE_DATA'],
            'REG_WRITE': cls.__constants['REG_WRITE'],
            'ACTION': cls.__constants['ACTION'],
            'FACTORY_RESET': cls.__constants['FACTORY_RESET'],
            'REBOOT': cls.__constants['REBOOT'],
            'SYNC_READ': cls.__constants['SYNC_READ'],
            'SYNC_WRITE': cls.__constants['SYNC_WRITE'],
            'BULK_READ': cls.__constants['BULK_READ'],
            'BULK_WRITE': cls.__constants['BULK_WRITE'],
            'MODEL_NUMBER': cls.__constants['MODEL_NUMBER'],
            'MODEL_INFORMATION': cls.__constants['MODEL_INFORMATION'],
            'FIRMWARE_VERSION': cls.__constants['FIRMWARE_VERSION'],
            'ID': cls.__constants['ID'],
            'BAUD_RATE': cls.__constants['BAUD_RATE'],
            'RETURN_DELAY_TIME': cls.__constants['RETURN_DELAY_TIME'],
            'DRIVE_MODE': cls.__constants['DRIVE_MODE'],
            'OPERATING_MODE': cls.__constants['OPERATING_MODE'],
            'SECONDARY_ID': cls.__constants['SECONDARY_ID'],
            'PROTOCOL_TYPE': cls.__constants['PROTOCOL_TYPE'],
            'HOMING_OFFSET': cls.__constants['HOMING_OFFSET'],
            'MOVING_THRESHOLD': cls.__constants['MOVING_THRESHOLD'],
            'TEMPERATURE_LIMIT': cls.__constants['TEMPERATURE_LIMIT'],
            'MAX_VOLTAGE_LIMIT': cls.__constants['MAX_VOLTAGE_LIMIT'],
            'MIN_VOLTAGE_LIMIT': cls.__constants['MIN_VOLTAGE_LIMIT'],
            'PWM_LIMIT': cls.__constants['PWM_LIMIT'],
            'CURRENT_LIMIT': cls.__constants['CURRENT_LIMIT'],
            'VELOCITY_LIMIT': cls.__constants['VELOCITY_LIMIT'],
            'MAX_POSITION_LIMIT': cls.__constants['MAX_POSITION_LIMIT'],
            'MIN_POSITION_LIMIT': cls.__constants['MIN_POSITION_LIMIT'],
            'TORQUE_ENABLE': cls.__constants['TORQUE_ENABLE'],
            'LED': cls.__constants['LED'],
            'STATUS_RETURN_LEVEL': cls.__constants['STATUS_RETURN_LEVEL'],
            'REGISTERED_INSTRUCTION': cls.__constants['REGISTERED_INSTRUCTION'],
            'HARDWARE_ERROR_STATUS': cls.__constants['HARDWARE_ERROR_STATUS'],
            'VELOCITY_I_GAIN': cls.__constants['VELOCITY_I_GAIN'],
            'VELOCITY_P_GAIN': cls.__constants['VELOCITY_P_GAIN'],
            'POSITION_D_GAIN': cls.__constants['POSITION_D_GAIN'],
            'POSITION_I_GAIN': cls.__constants['POSITION_I_GAIN'],
            'POSITION_P_GAIN': cls.__constants['POSITION_P_GAIN'],
            'FEEDFORWARD_2_GAIN': cls.__constants['FEEDFORWARD_2_GAIN'],
            'FEEDFORWARD_1_GAIN': cls.__constants['FEEDFORWARD_1_GAIN'],
            'BUS_WATCHDOG': cls.__constants['BUS_WATCHDOG'],
            'GOAL_PWM': cls.__constants['GOAL_PWM'],
            'GOAL_CURRENT': cls.__constants['GOAL_CURRENT'],
            'GOAL_VELOCITY': cls.__constants['GOAL_VELOCITY'],
            'PROFILE_ACCELERATION': cls.__constants['PROFILE_ACCELERATION'],
            'PROFILE_VELOCITY': cls.__constants['PROFILE_VELOCITY'],
            'GOAL_POSITION': cls.__constants['GOAL_POSITION'],
            'REALTIME_TICK': cls.__constants['REALTIME_TICK'],
            'MOVING': cls.__constants['MOVING'],
            'PRESENT_PWM': cls.__constants['PRESENT_PWM'],
            'PRESENT_CURRENT': cls.__constants['PRESENT_CURRENT'],
            'PRESENT_VELOCITY': cls.__constants['PRESENT_VELOCITY'],
            'PRESENT_POSITION': cls.__constants['PRESENT_POSITION'],
            'PRESENT_INPUT_VOLTAGE': cls.__constants['PRESENT_INPUT_VOLTAGE'],
            'PRESENT_TEMPERATURE': cls.__constants['PRESENT_TEMPERATURE'],
        }

    @property
    def PING(self):
        """Message constant 'PING'."""
        return Metaclass_DynamixelController.__constants['PING']

    @property
    def READ_DATA(self):
        """Message constant 'READ_DATA'."""
        return Metaclass_DynamixelController.__constants['READ_DATA']

    @property
    def WRITE_DATA(self):
        """Message constant 'WRITE_DATA'."""
        return Metaclass_DynamixelController.__constants['WRITE_DATA']

    @property
    def REG_WRITE(self):
        """Message constant 'REG_WRITE'."""
        return Metaclass_DynamixelController.__constants['REG_WRITE']

    @property
    def ACTION(self):
        """Message constant 'ACTION'."""
        return Metaclass_DynamixelController.__constants['ACTION']

    @property
    def FACTORY_RESET(self):
        """Message constant 'FACTORY_RESET'."""
        return Metaclass_DynamixelController.__constants['FACTORY_RESET']

    @property
    def REBOOT(self):
        """Message constant 'REBOOT'."""
        return Metaclass_DynamixelController.__constants['REBOOT']

    @property
    def SYNC_READ(self):
        """Message constant 'SYNC_READ'."""
        return Metaclass_DynamixelController.__constants['SYNC_READ']

    @property
    def SYNC_WRITE(self):
        """Message constant 'SYNC_WRITE'."""
        return Metaclass_DynamixelController.__constants['SYNC_WRITE']

    @property
    def BULK_READ(self):
        """Message constant 'BULK_READ'."""
        return Metaclass_DynamixelController.__constants['BULK_READ']

    @property
    def BULK_WRITE(self):
        """Message constant 'BULK_WRITE'."""
        return Metaclass_DynamixelController.__constants['BULK_WRITE']

    @property
    def MODEL_NUMBER(self):
        """Message constant 'MODEL_NUMBER'."""
        return Metaclass_DynamixelController.__constants['MODEL_NUMBER']

    @property
    def MODEL_INFORMATION(self):
        """Message constant 'MODEL_INFORMATION'."""
        return Metaclass_DynamixelController.__constants['MODEL_INFORMATION']

    @property
    def FIRMWARE_VERSION(self):
        """Message constant 'FIRMWARE_VERSION'."""
        return Metaclass_DynamixelController.__constants['FIRMWARE_VERSION']

    @property
    def ID(self):
        """Message constant 'ID'."""
        return Metaclass_DynamixelController.__constants['ID']

    @property
    def BAUD_RATE(self):
        """Message constant 'BAUD_RATE'."""
        return Metaclass_DynamixelController.__constants['BAUD_RATE']

    @property
    def RETURN_DELAY_TIME(self):
        """Message constant 'RETURN_DELAY_TIME'."""
        return Metaclass_DynamixelController.__constants['RETURN_DELAY_TIME']

    @property
    def DRIVE_MODE(self):
        """Message constant 'DRIVE_MODE'."""
        return Metaclass_DynamixelController.__constants['DRIVE_MODE']

    @property
    def OPERATING_MODE(self):
        """Message constant 'OPERATING_MODE'."""
        return Metaclass_DynamixelController.__constants['OPERATING_MODE']

    @property
    def SECONDARY_ID(self):
        """Message constant 'SECONDARY_ID'."""
        return Metaclass_DynamixelController.__constants['SECONDARY_ID']

    @property
    def PROTOCOL_TYPE(self):
        """Message constant 'PROTOCOL_TYPE'."""
        return Metaclass_DynamixelController.__constants['PROTOCOL_TYPE']

    @property
    def HOMING_OFFSET(self):
        """Message constant 'HOMING_OFFSET'."""
        return Metaclass_DynamixelController.__constants['HOMING_OFFSET']

    @property
    def MOVING_THRESHOLD(self):
        """Message constant 'MOVING_THRESHOLD'."""
        return Metaclass_DynamixelController.__constants['MOVING_THRESHOLD']

    @property
    def TEMPERATURE_LIMIT(self):
        """Message constant 'TEMPERATURE_LIMIT'."""
        return Metaclass_DynamixelController.__constants['TEMPERATURE_LIMIT']

    @property
    def MAX_VOLTAGE_LIMIT(self):
        """Message constant 'MAX_VOLTAGE_LIMIT'."""
        return Metaclass_DynamixelController.__constants['MAX_VOLTAGE_LIMIT']

    @property
    def MIN_VOLTAGE_LIMIT(self):
        """Message constant 'MIN_VOLTAGE_LIMIT'."""
        return Metaclass_DynamixelController.__constants['MIN_VOLTAGE_LIMIT']

    @property
    def PWM_LIMIT(self):
        """Message constant 'PWM_LIMIT'."""
        return Metaclass_DynamixelController.__constants['PWM_LIMIT']

    @property
    def CURRENT_LIMIT(self):
        """Message constant 'CURRENT_LIMIT'."""
        return Metaclass_DynamixelController.__constants['CURRENT_LIMIT']

    @property
    def VELOCITY_LIMIT(self):
        """Message constant 'VELOCITY_LIMIT'."""
        return Metaclass_DynamixelController.__constants['VELOCITY_LIMIT']

    @property
    def MAX_POSITION_LIMIT(self):
        """Message constant 'MAX_POSITION_LIMIT'."""
        return Metaclass_DynamixelController.__constants['MAX_POSITION_LIMIT']

    @property
    def MIN_POSITION_LIMIT(self):
        """Message constant 'MIN_POSITION_LIMIT'."""
        return Metaclass_DynamixelController.__constants['MIN_POSITION_LIMIT']

    @property
    def TORQUE_ENABLE(self):
        """Message constant 'TORQUE_ENABLE'."""
        return Metaclass_DynamixelController.__constants['TORQUE_ENABLE']

    @property
    def LED(self):
        """Message constant 'LED'."""
        return Metaclass_DynamixelController.__constants['LED']

    @property
    def STATUS_RETURN_LEVEL(self):
        """Message constant 'STATUS_RETURN_LEVEL'."""
        return Metaclass_DynamixelController.__constants['STATUS_RETURN_LEVEL']

    @property
    def REGISTERED_INSTRUCTION(self):
        """Message constant 'REGISTERED_INSTRUCTION'."""
        return Metaclass_DynamixelController.__constants['REGISTERED_INSTRUCTION']

    @property
    def HARDWARE_ERROR_STATUS(self):
        """Message constant 'HARDWARE_ERROR_STATUS'."""
        return Metaclass_DynamixelController.__constants['HARDWARE_ERROR_STATUS']

    @property
    def VELOCITY_I_GAIN(self):
        """Message constant 'VELOCITY_I_GAIN'."""
        return Metaclass_DynamixelController.__constants['VELOCITY_I_GAIN']

    @property
    def VELOCITY_P_GAIN(self):
        """Message constant 'VELOCITY_P_GAIN'."""
        return Metaclass_DynamixelController.__constants['VELOCITY_P_GAIN']

    @property
    def POSITION_D_GAIN(self):
        """Message constant 'POSITION_D_GAIN'."""
        return Metaclass_DynamixelController.__constants['POSITION_D_GAIN']

    @property
    def POSITION_I_GAIN(self):
        """Message constant 'POSITION_I_GAIN'."""
        return Metaclass_DynamixelController.__constants['POSITION_I_GAIN']

    @property
    def POSITION_P_GAIN(self):
        """Message constant 'POSITION_P_GAIN'."""
        return Metaclass_DynamixelController.__constants['POSITION_P_GAIN']

    @property
    def FEEDFORWARD_2_GAIN(self):
        """Message constant 'FEEDFORWARD_2_GAIN'."""
        return Metaclass_DynamixelController.__constants['FEEDFORWARD_2_GAIN']

    @property
    def FEEDFORWARD_1_GAIN(self):
        """Message constant 'FEEDFORWARD_1_GAIN'."""
        return Metaclass_DynamixelController.__constants['FEEDFORWARD_1_GAIN']

    @property
    def BUS_WATCHDOG(self):
        """Message constant 'BUS_WATCHDOG'."""
        return Metaclass_DynamixelController.__constants['BUS_WATCHDOG']

    @property
    def GOAL_PWM(self):
        """Message constant 'GOAL_PWM'."""
        return Metaclass_DynamixelController.__constants['GOAL_PWM']

    @property
    def GOAL_CURRENT(self):
        """Message constant 'GOAL_CURRENT'."""
        return Metaclass_DynamixelController.__constants['GOAL_CURRENT']

    @property
    def GOAL_VELOCITY(self):
        """Message constant 'GOAL_VELOCITY'."""
        return Metaclass_DynamixelController.__constants['GOAL_VELOCITY']

    @property
    def PROFILE_ACCELERATION(self):
        """Message constant 'PROFILE_ACCELERATION'."""
        return Metaclass_DynamixelController.__constants['PROFILE_ACCELERATION']

    @property
    def PROFILE_VELOCITY(self):
        """Message constant 'PROFILE_VELOCITY'."""
        return Metaclass_DynamixelController.__constants['PROFILE_VELOCITY']

    @property
    def GOAL_POSITION(self):
        """Message constant 'GOAL_POSITION'."""
        return Metaclass_DynamixelController.__constants['GOAL_POSITION']

    @property
    def REALTIME_TICK(self):
        """Message constant 'REALTIME_TICK'."""
        return Metaclass_DynamixelController.__constants['REALTIME_TICK']

    @property
    def MOVING(self):
        """Message constant 'MOVING'."""
        return Metaclass_DynamixelController.__constants['MOVING']

    @property
    def PRESENT_PWM(self):
        """Message constant 'PRESENT_PWM'."""
        return Metaclass_DynamixelController.__constants['PRESENT_PWM']

    @property
    def PRESENT_CURRENT(self):
        """Message constant 'PRESENT_CURRENT'."""
        return Metaclass_DynamixelController.__constants['PRESENT_CURRENT']

    @property
    def PRESENT_VELOCITY(self):
        """Message constant 'PRESENT_VELOCITY'."""
        return Metaclass_DynamixelController.__constants['PRESENT_VELOCITY']

    @property
    def PRESENT_POSITION(self):
        """Message constant 'PRESENT_POSITION'."""
        return Metaclass_DynamixelController.__constants['PRESENT_POSITION']

    @property
    def PRESENT_INPUT_VOLTAGE(self):
        """Message constant 'PRESENT_INPUT_VOLTAGE'."""
        return Metaclass_DynamixelController.__constants['PRESENT_INPUT_VOLTAGE']

    @property
    def PRESENT_TEMPERATURE(self):
        """Message constant 'PRESENT_TEMPERATURE'."""
        return Metaclass_DynamixelController.__constants['PRESENT_TEMPERATURE']


class DynamixelController(metaclass=Metaclass_DynamixelController):
    """
    Message class 'DynamixelController'.

    Constants:
      PING
      READ_DATA
      WRITE_DATA
      REG_WRITE
      ACTION
      FACTORY_RESET
      REBOOT
      SYNC_READ
      SYNC_WRITE
      BULK_READ
      BULK_WRITE
      MODEL_NUMBER
      MODEL_INFORMATION
      FIRMWARE_VERSION
      ID
      BAUD_RATE
      RETURN_DELAY_TIME
      DRIVE_MODE
      OPERATING_MODE
      SECONDARY_ID
      PROTOCOL_TYPE
      HOMING_OFFSET
      MOVING_THRESHOLD
      TEMPERATURE_LIMIT
      MAX_VOLTAGE_LIMIT
      MIN_VOLTAGE_LIMIT
      PWM_LIMIT
      CURRENT_LIMIT
      VELOCITY_LIMIT
      MAX_POSITION_LIMIT
      MIN_POSITION_LIMIT
      TORQUE_ENABLE
      LED
      STATUS_RETURN_LEVEL
      REGISTERED_INSTRUCTION
      HARDWARE_ERROR_STATUS
      VELOCITY_I_GAIN
      VELOCITY_P_GAIN
      POSITION_D_GAIN
      POSITION_I_GAIN
      POSITION_P_GAIN
      FEEDFORWARD_2_GAIN
      FEEDFORWARD_1_GAIN
      BUS_WATCHDOG
      GOAL_PWM
      GOAL_CURRENT
      GOAL_VELOCITY
      PROFILE_ACCELERATION
      PROFILE_VELOCITY
      GOAL_POSITION
      REALTIME_TICK
      MOVING
      PRESENT_PWM
      PRESENT_CURRENT
      PRESENT_VELOCITY
      PRESENT_POSITION
      PRESENT_INPUT_VOLTAGE
      PRESENT_TEMPERATURE
    """

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)
