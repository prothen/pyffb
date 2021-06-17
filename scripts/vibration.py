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
TIME_TO_WAIT_IN_SECONDS = 10


if __name__ == "__main__":
    print('Start module test to configure settings and overrides.')

    interface = ffb.interface.Interface(debug_is_enabled=True)

    try:
        interface.connect()
        print('Example usage of API for Axis.X:')

        print('Override: Allow vibration override...')
        # Enable vibration
        interface.enable_override(
            axis=Axis.X,
            command_type=OverrideControl.VibrationOverride)

        # Set vibration
        print('Setting: Set vibration active...')
        interface.update_setting(
            axis=Axis.X,
            command_type=SettingsControl.VibrationActive,
            value=1)

        frequency = 10.
        dt_in_ms = int(1 / frequency * 1000)
        print('Setting: Set vibration frequency to {} Hz ({}ms)'.format(
            frequency, dt_in_ms))
        interface.update_setting(
            axis=Axis.X,
            command_type=SettingsControl.VibrationSpeedForFastVibrations,
            value=dt_in_ms)

        print('Wait for 5 seconds before disconnecting.')
        time.sleep(5)

        print('Setting: Set vibration inactive...')
        interface.update_setting(
            axis=Axis.X,
            command_type=SettingsControl.VibrationActive,
            value=0)

        print('--> [x] Finished successfully')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)
    interface.exit()
    sys.exit()

