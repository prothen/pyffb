#!/usr/bin/env python
"""
    Interface to Brunner CLS-E force feedback joystick.

    Author: Philipp Rothenhäusler, Stockholm 2021

"""

import sys
import time

from ffb.protocol import *

import ffb.interface


if __name__ == "__main__":
    print('Start module test to configure friction.')

    interface = ffb.interface.Interface(debug_is_enabled=True)

    try:
        interface.connect()
        print('Example usage of API for Axis.X:')

        print('Set friction value...')
        interface.update_setting(
            axis=Axis.X,
            command_type=SettingsControl.FrictionValue,
            value=65535)

        print('Wait for 5 seconds before disconnecting.')
        time.sleep(5)
        print('--> [x] Finished successfully')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)
    interface.exit()
    sys.exit()

