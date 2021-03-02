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
    print('Start module test')

    interface = ffb.interface.Interface()
    interface.connect()

    try:
        interface.connect()
        interface.launch_receiver_thread()

        t0 = time.time()
        t = 0
        while (t < 20) and interface.is_active:
            ## Send single actuation force vector (-1, 1)
            # interface.actuate(x=0, y=0, safe=False)

            ## Run time varying actuation test (sine)
            # interface.actuate_test(t)

            ## Read joystick position
            position = interface.get_state()
            print("{:.2f} | Position: {}".format(t, position))
            time.sleep(0.1)
            t = time.time() - t0
        print('--> [x] Completed module test')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)
    interface.exit()

