#!/usr/bin/env python
"""
    Interface to Brunner CLS-E force feedback joystick.

    Author: Philipp Rothenhäusler, Stockholm 2020

"""

import sys
import time

from ffb.protocol import *

import ffb.interface


if __name__ == "__main__":
    print('Start module test for reading digital inputs on Axis.X.')

    interface = ffb.interface.Interface(debug_is_enabled=True)

    try:
        interface.connect()

        deadline = time.time() + 10
        while deadline > time.time():
            data = interface.read_data(
                    axis=Axis.X,
                    command_type=DataRead.DigitalInputs)
            print('Axis.X: ', data)

            data = interface.read_data(
                    axis=Axis.Y,
                    command_type=DataRead.DigitalInputs)
            print('Axis.Y: ', data)

            time.sleep(.5)

        print('--> [x] Finished successfully')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)
    interface.exit()
    sys.exit()

