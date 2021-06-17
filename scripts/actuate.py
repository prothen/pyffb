#!/usr/bin/env python
"""
    Interface to Brunner CLS-E force feedback joystick.

    Author: Philipp Rothenhäusler, Stockholm 2020

"""

import time


import ffb.interface


if __name__ == "__main__":
    print('Start module test for external force actuation.')

    interface = ffb.interface.Interface(debug_is_enabled=True)
    interface.connect()

    try:
        interface.launch_receiver_thread()

        t0 = time.time()
        t = 0
        while (t < 20) and interface.is_active:
            # Send single actuation force vector (-1, 1)
            # interface.actuate_safe(x=-1, y=1, safe=False)

            # Send time-varying actuation test (sine)
            interface.actuate_test(t)

            # Read joystick states
            all_states = interface.state
            position_xy = interface.get_axes_xy()

            print("{:.2f} | All axis states: {}".format(t, all_states))
            print("{:.2f} | Position (xy): {}".format(t, position_xy))

            time.sleep(0.1)
            t = time.time() - t0
        print('--> [x] Completed module test')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)

    interface.exit()

