#!/usr/bin/env python
"""
    Interface to Brunner CLS-E force feedback joystick.

    Author: Philipp Rothenhäusler, Stockholm 2021

"""

import sys
import time

from ffb.protocol import *

import ffb.interface


TEST_SPEED = 15
TIME_TO_WAIT_IN_SECONDS = 5


def move_to_position(
        interface,
        value,
        wait_for=TIME_TO_WAIT_IN_SECONDS):
    """ Convenience script to update reference setpoint for axis. """
    print('Set autopilot position: {}'.format(value))
    # Set autopilot reference position
    interface.update_setting(
        axis=Axis.X,
        command_type=SettingsControl.AutopilotPosition,
        value=numpy.array(value))

    print('Wait for {} seconds ...'.format(wait_for))
    time.sleep(wait_for)


if __name__ == "__main__":
    print('Start module test to configure autopilot...')

    interface = ffb.interface.Interface(debug_is_enabled=True)

    try:
        interface.connect()
        print('Example usage of API for Axis.X:')

        print('Override: Allow autopilot enable...')
        interface.enable_override(
            axis=Axis.X,
            command_type=OverrideControl.AutopilotEnableOverride)

        print('Override: Allow autopilot position override...')
        interface.enable_override(
            axis=Axis.X,
            command_type=OverrideControl.AutopilotPositionOverride)

        print('Setting: Set autopilot enabled...')
        interface.update_setting(
            axis=Axis.X,
            command_type=SettingsControl.AutopilotEnable,
            value=1)

        print('Setting: Set autopilot speed...')
        # Set autopilot speed
        # Valid range: 5 - 65535 (Recommended start: < 10)
        interface.update_setting(
            axis=Axis.X,
            command_type=SettingsControl.AutopilotSpeed,
            value=TEST_SPEED)

        print('Setting: Set autopilot force...')
        # Set autopilot force
        # Valid range: 1 - 65535 (Recommended start: ~1500)
        interface.update_setting(
            axis=Axis.X,
            command_type=SettingsControl.AutopilotForce,
            value=65535)

        # Execute test script
        print('Test: Move joystick forward->backward->origin.')
        move_to_position(interface, 1)
        move_to_position(interface, -1)
        move_to_position(interface, 0)

        print('Setting: Set autopilot disabled...')
        interface.update_setting(
            axis=Axis.X,
            command_type=SettingsControl.AutopilotEnable,
            value=0)

        print('--> [x] Finished successfully')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)
    interface.exit()
    sys.exit()
