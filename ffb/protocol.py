#!/usr/bin/env python
"""
    Command, axis and message definitions.

    Author: Philipp Rothenhäusler, Stockholm 2020

"""


import enum


class Command(enum.IntEnum):
    DataRead = 0xD0
    RemoteControlConfig = 0xD8
    StateControl = 0xCD
    SettingsControl = 0xCE
    ProfileControl = 0xCF
    IO2CANGatewayRead = 0xD2
    IO2CANGatewayWrite = 0xD6
    IO2CANGatewaySetTimeout = 0xD7


class Axis(enum.IntEnum):
    X = 0
    Y = 1


class Measurement(enum.IntEnum):
    Position = 0
    PositionNormalized = 1
    AppliedForce = 2
    AppliedForceNormalized = 3
    DigitalInputs = 4
    AnalogInputs = 5
    VirtualForce = 6
    Range = 7


# Note: Other measurements are added only on demand

measurement2format = dict()
measurement2format[Measurement.Position] = "h"
measurement2format[Measurement.PositionNormalized] = "f"
measurement2format[Measurement.AppliedForce] = "i"
measurement2format[Measurement.AppliedForceNormalized] = "f"
measurement2format[Measurement.Range] = "iiii"


axis2axis_id = dict()
axis2axis_id[Axis.X] = 0x01
axis2axis_id[Axis.Y] = 0x02


measurement2data_id = dict()
measurement2data_id[Measurement.Position] = 0x10
measurement2data_id[Measurement.PositionNormalized] = 0x11
measurement2data_id[Measurement.AppliedForce] = 0x20
measurement2data_id[Measurement.AppliedForceNormalized] = 0x21
measurement2data_id[Measurement.Range] = 0x60


