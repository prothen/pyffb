#!/usr/bin/env python
"""
    Interface to Brunner CLS-E force feedback joystick.

    Author: Philipp Rothenhäusler, Stockholm 2020

"""

import sys
import time
import enum

from ffb.protocol import *

import ffb.interface


## TODO: Test cases for different use cases


class TestCase(enum.IntEnum):
    DataRead = 1
    SettingsControl = 2
    OVerrideControl = 3

TEST_SPEED = 15

TIME_TO_WAIT_IN_SECONDS = 5

if __name__ == "__main__":
    print('Start module test')

    interface = ffb.interface.Interface(debug_is_enabled=True)

    try:
        interface.connect()

        ## Test SettingControl
        if True:

            print('Set friction...')
            interface.update_setting(
                axis=Axis.X,
                command_type=SettingsControl.FrictionValue,
                value=65535)
            time.sleep(10)
            aoeueuo
            print('Set autopilot speed...')
            # Set autopilot speed (Recommended < 10 to start)
            interface.update_setting(
                axis=Axis.X,
                command_type=SettingsControl.AutopilotSpeed,
                value=TEST_SPEED)

            print('Set autopilot force...')
            # Set autopilot force (Recommended <1500)
            interface.update_setting(
                axis=Axis.X,
                command_type=SettingsControl.AutopilotForce,
                value=1500)

            print('Set autopilot position...')
            # Set autopilot reference position
            interface.update_setting(
                axis=Axis.X,
                command_type=SettingsControl.AutopilotPosition,
                value=numpy.array(.5).astype(numpy.float32))

            print('Set autopilot setting active for axis...')
            interface.update_setting(
                axis=Axis.X,
                command_type=SettingsControl.AutopilotEnable,
                value=1)

            # Set vibration
            print('Set vibration')
            interface.update_setting(
                axis=Axis.X,
                command_type=SettingsControl.VibrationActive,
                value=1)

            frequency = 10.
            dt_in_ms = int(1 / frequency * 1000)
            print('Set vibration frequency to {} Hz ({}ms)'.format(frequency, dt_in_ms))
            interface.update_setting(
                axis=Axis.X,
                command_type=SettingsControl.VibrationSpeedForFastVibrations,
                value=dt_in_ms)

            ## Test OverrideControl
            print('Confirm overrides for axis...')
            print('Enable autopilot override...')
            # Enable vibration
            interface.enable_override(
                axis=Axis.X,
                command_type=OverrideControl.AutopilotEnableOverride)

            print('Enable autopilot position override...')
            # Enable vibration
            interface.enable_override(
                axis=Axis.X,
                command_type=OverrideControl.AutopilotPositionOverride)

            # Enable vibration
            interface.enable_override(
                axis=Axis.X,
                command_type=OverrideControl.VibrationOverride)

            print('Wait for {} seconds ...'.format(TIME_TO_WAIT_IN_SECONDS))
            time.sleep(TIME_TO_WAIT_IN_SECONDS)

            print('Set autopilot speed...')
            # Set autopilot speed (Recommended < 10 to start)
            interface.update_setting(
                axis=Axis.X,
                command_type=SettingsControl.AutopilotSpeed,
                value=TEST_SPEED)

            interface.update_setting(
                axis=Axis.X,
                command_type=SettingsControl.AutopilotPosition,
                value=numpy.array(-1).astype(numpy.float32))

            print('Wait for {} seconds ...'.format(TIME_TO_WAIT_IN_SECONDS))
            time.sleep(TIME_TO_WAIT_IN_SECONDS)

            interface.update_setting(
                axis=Axis.X,
                command_type=SettingsControl.AutopilotPosition,
                value=numpy.array(1).astype(numpy.float32))

            print('Wait for {} seconds ...'.format(TIME_TO_WAIT_IN_SECONDS))
            time.sleep(TIME_TO_WAIT_IN_SECONDS)

            interface.update_setting(
                axis=Axis.X,
                command_type=SettingsControl.AutopilotEnable,
                value=0)

        print('--> [x] Finished successfully')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)
    interface.exit()
    sys.exit()

