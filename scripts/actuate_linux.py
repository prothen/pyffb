#!/usr/bin/env python
"""
    Interface to Brunner CLS-E force feedback joystick.

    Author: Philipp Rothenhäusler, Stockholm 2020

"""

import time


import ffb.interface


if __name__ == "__main__":
    print('Start module test for external force actuation.')

    interface = ffb.interface.Interface(config_name="linux", debug_is_enabled=True)
    interface.connect()

    try:
        # interface.launch_receiver_thread()

        t0 = time.time()
        t = 0
        while (t < 10) and interface.actuation_stream_is_active():

            # Test automatic idle disengagement after 5 seconds
            if t>5:
                continue

            # Send time-varying actuation test (sine)
            if interface.actuate_test(t):
                # Read joystick states
                all_states = interface.state
                position_xy = interface.get_axes_xy()

                # Rate limit to 100 Hz
                time.sleep(0.01)
            t = time.time() - t0
        print('--> [x] Completed module test')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)

    interface.exit()

