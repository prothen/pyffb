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

    # Test setting retrieval
    if False:
            # Position
            print('Position')
            interface.read_settings(Axis.X, Measurement.Position)
            interface.read_settings(Axis.Y, Measurement.Position)
            # PositionNormalized
            print('PositionNormalized')
            interface.read_settings(Axis.X, Measurement.PositionNormalized)
            interface.read_settings(Axis.Y, Measurement.PositionNormalized)
            # AppliedForces
            print('AppliedForces')
            interface.read_settings(Axis.X, Measurement.AppliedForce)
            interface.read_settings(Axis.Y, Measurement.AppliedForce)
            # AppliedForcesNormalized
            print('AppliedForcesNormalized')
            interface.read_settings(Axis.X, Measurement.AppliedForceNormalized)
            interface.read_settings(Axis.Y, Measurement.AppliedForceNormalized)
            print('Range')
            interface.read_settings(Axis.X, Measurement.Range)
            interface.read_settings(Axis.Y, Measurement.Range)
            interface.exit()
            sys.exit()

    try:
        interface.connect()
        print("Launch receiver callback")

        interface.launch_receiver_thread()

        t0 = time.time()
        t = 0
        while (t < 20) and interface.is_active:
            # interface.actuate(safe=False)
            interface.actuate_test(t)
            position = interface.get_state()
            print("{:.2f} | Position: {}".format(t, position))
            time.sleep(0.1)
            t = time.time() - t0
        print('--> [x] Completed module test')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)
    interface.exit()

