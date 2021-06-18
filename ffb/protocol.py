#!/usr/bin/env python
"""
    Command, axis and message definitions.

    For reference of the format string used for struct creation see

    https://docs.python.org/3/library/struct.html

    Basics:
        float 'f'
        signed / unsigned byte 'b' / 'B'
        signed / unsigned int16 'h' / 'H'
        signed / unsigned int32 'i' / 'I'


    Author: Philipp Rothenhäusler, Stockholm 2020

"""


import enum
import attr
import numpy
import typing


# Response elements
# Packetlength (uint16_t) |
# (optional) Identifier (uint32_t) |
# Status (uint8_t)
# Response value (... see CommandType.format)
RESPONSE_FORMAT = "HB"

# In case of Messageidentifier uncomment below
# RESPONSE_FORMAT = "HIB"


@attr.s
class CommandDefinition:
    # Identifier from CLS2Sim protocol v2
    identifier = attr.ib(type=int)
    # Common format (implied additional arguments
    request_format = attr.ib(type=str)
    # Define request elements
    request_labels = attr.ib(type=typing.List[str])
    # Response format (excluding CommandType specifics to be appended)
    response_format = attr.ib(type=str)
    # Define response elements
    response_labels = attr.ib(type=typing.List[str])

    # Decide whether supplied value is encoded for request
    sends_value = attr.ib(default=False, type=bool)
    # Decide whether some received value is decoded from response
    receives_value = attr.ib(default=False, type=bool)

    # TODO: Possibly add different error style classes to handle varying cases


class Command:

    # Reading data: (command, axis, data_id)
    StateControl = CommandDefinition(
        identifier=0xCD,
        request_format="III",
        request_labels=["Command", "Axis",
                        "StateCommand"],
        response_format="HBHHH",
        response_labels=["PacketLength", "Status",
                         "DeviceId", "DeviceState", "DeviceTemperature"],
        receives_value=True,)
    # Reading data: (command, axis, data_id)
    DataRead = CommandDefinition(
        identifier=0xD0,
        request_format="III",
        request_labels=["Command", "Axis",
                        "DataId"],
        response_format="HBH",
        response_labels=["PacketLength", "Status",
                         "DeviceId", "Data"],
        receives_value=True,)

    # TODO: Type specification
    # RemoteControlConfig = CommandDefinition(
    #     identifier=0xD8)

    # TODO: Type specification
    # StateControl = CommandDefinition(
    #     identifier=0xCD)

    # TODO: Type specification
    SettingsControl = CommandDefinition(
        identifier=0xCE,
        request_format="iii",
        request_labels=["Command", "Axis",
                        "SettingId", "SettingValue"],
        response_format="HB",
        response_labels=["PacketLength", "Status"],
        sends_value=True)

    # ProfileControl = CommandDefinition(
    #     identifier=0xCF)
    # IO2CANGatewayRead = CommandDefinition(
    #     identifier=0xD2)
    # IO2CANGatewayWrite = CommandDefinition(
    #     identifier=0xD6)
    # IO2CANGatewaySetTimeout = CommandDefinition(
    #     identifier=0xD7)

    #
    OverrideControl = CommandDefinition(
        identifier=0xD1,
        request_format="III",
        request_labels=["Command", "Axis",
                        "OverrideId"],
        response_format="HB",
        response_labels=["PacketLength", "Status"],)


class Axis(enum.IntEnum):
    """ Selecting X as elevator and y as aileron yields REP103 body frame. """
    # Elevator (forward positive)
    X = 0x01
    # Aileron (left positive)
    Y = 0x02


@attr.s
class ValueConstraints:
    """ Scalar value constraint convenience object. """
    # TODO: Auto convert any type to numpy.ndarray
    min = attr.ib(default=-numpy.inf) #, type=numpy.ndarray)
    max = attr.ib(default=numpy.inf) # , type=numpy.ndarray)
    ##
    # is_type = attr.ib(default=None)

    def is_satisfied(self, value: numpy.ndarray):
        # Check type instance
        return self.min <= value <= self.max


@attr.s
class CommandType:
    # Define identifier for command interface
    identifier = attr.ib(type=int)
    # Define command type specific return or send format
    format = attr.ib(default=None, type=typing.Optional[str])
    # Value range and type specifications
    value_range = attr.ib(default=ValueConstraints(), type=ValueConstraints)


class DataRead:
    Position = CommandType(format="h", identifier=0x10)
    PositionNormalized = CommandType(format="f", identifier=0x11)
    AppliedForce = CommandType(format="i", identifier=0x20)
    AppliedForceNormalized = CommandType(format="f", identifier=0x21)
    # LEGACY
    DigitalInputs = CommandType(format="i", identifier=0x30)
    # TODO: Digital input sequence is variable for extended command
    #       handle with if condition in command.receives_value
    DigitalInputsExtended = CommandType(format="H", identifier=0x31)
    # AnalogInputs = CommandEntry(format="h", identifier=0x10)
    # VirtualForce = CommandEntry(format="h", identifier=0x10)
    Range = CommandType(format="iiii", identifier=0x60)


class SettingsControl:
    ForceProfile = CommandType(
        format="H", identifier=0x10,
        value_range=ValueConstraints(min=5, max=99999))
    ForceScaleFactor = CommandType(
        format="H", identifier=0x20,
        value_range=ValueConstraints(min=20, max=999))
    FrictionValue = CommandType(
        format="H", identifier=0x25,
        value_range=ValueConstraints(min=20, max=65535))
    FrictionValueEng = CommandType(
        format="f", identifier=0x26,
        value_range=ValueConstraints(min=.1))

    PositionWindow = CommandType(
        format="I", identifier=0x27,
        value_range=ValueConstraints(min=0, max=2**32))
    TargetTorque = CommandType(
        format="h", identifier=0x28,
        value_range=ValueConstraints(min=0, max=65535))
    ProfileMode = CommandType(
        format="B", identifier=0x30,
        value_range=ValueConstraints(min=0, max=1))
    MoveBackEnable = CommandType(
        format="B", identifier=0x40,
        value_range=ValueConstraints(min=0, max=1))

    AutopilotForce = CommandType(
        format="H", identifier=0x50,
        value_range=ValueConstraints(min=5, max=99999))
    AutopilotSpeed = CommandType(
        format="H", identifier=0x60,
        value_range=ValueConstraints(min=5, max=10))
    AutopilotEnable = CommandType(
        format="B", identifier=0x70,
        value_range=ValueConstraints(min=0, max=1))
    AutopilotPosition = CommandType(
        format="f", identifier=0x80,
        value_range=ValueConstraints(min=-1, max=1))

    VibrationActive = CommandType(
        format="B", identifier=0xC0,
        value_range=ValueConstraints(min=0, max=1))
    VibrationSpeedForFastVibrations = CommandType(
        format="I", identifier=0xC1,
        value_range=ValueConstraints(min=10))


class OverrideControl:
    """

        Note:
                To disable all override send 0x00 as axis.
    """

    ScaleFactorOverride = CommandType(identifier=0x01)
    TrimPositionOverride = CommandType(identifier=0x02)
    AutopilotEnableOverride = CommandType(identifier=0x03)
    AutopilotPositionOverride = CommandType(identifier=0x04)
    VibrationOverride = CommandType(identifier=0x05)
