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

    try:
        interface.connect()

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
        print('--> [x] Finished successfully')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)
    interface.exit()
    sys.exit()

