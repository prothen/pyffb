#!/usr/bin/env python
"""
    Interface to Brunner CLS-E force feedback joystick.

    Todo:
        - add callback to receiver thread to for example forward to another interface


    Author: Philipp Rothenhäusler, Stockholm 2020

"""

import os
import sys
import time
import attr
import yaml
import numpy
import typing
import socket
import struct
import signal
import threading


from ffb.protocol import *
from ffb.custom_thread import *


@attr.s
class Interface:
    # Variables
    force = attr.ib(default=None, type=typing.Optional[numpy.ndarray])
    state = attr.ib(default=None, type=typing.Optional[numpy.ndarray])

    _receiver_thread_active = attr.ib(default=False, type=bool)

    # Configuration parameters
    config_name = attr.ib(default="default", type=str)
    config_path = attr.ib(default=None, type=typing.Optional[str])
    host = attr.ib(default=None, type=typing.Optional[str])
    port = attr.ib(default=None, type=typing.Optional[int])
    buffer_size = attr.ib(default=None, type=typing.Optional[int])
    timeout = attr.ib(default=None, type=typing.Optional[float])
    socket = attr.ib(default=None, type=typing.Optional[socket.socket])
    force_max = attr.ib(default=None, type=typing.Optional[float])

    def __attrs_post_init__(self):
        print('Custom initialisation after parsing')

        self._thread_shutdown_request = threading.Event()

        signal.signal(signal.SIGINT, self._exit_routine)
        signal.signal(signal.SIGTERM, self._exit_routine)

        self._initialise_from_yaml()
        self._initialise_variables()

    def _exit_routine(self, sig, frame):
        print('Requested shutdown')
        self._thread_shutdown_request.set()
        self._receiver_thread_active = False

    def _initialise_from_yaml(self):
        config_name = self.config_name
        cwd = os.path.dirname(os.path.abspath(__file__))
        path = "{}/../config".format(cwd) if self.config_path is None else self.config_path
        print("Current absolute path: ", cwd)
        try:
            if config_name is not None:

                with open('{}/{}.yaml'.format(
                    path,
                    config_name), 'r') as stream:
                    try:
                        config = yaml.safe_load(stream)
                    except yaml.YAMLError as e:
                        print('Configuration parsing failed: Encountered\n', e)
                self.__dict__.update(config)
        except:
            raise RuntimeError()
        print('Configuration parsing finished successfully. (Loaded {})'.format(config_name))

    def _initialise_variables(self):
        self.force = numpy.zeros(8, dtype=numpy.int32)
        self.state = numpy.zeros(8, dtype=numpy.float)

    def connect(self):
        print('Connecting...')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(self.timeout)
        self.socket = sock
        print('--> [x] Connected')

    def _print_message_fields(self, message):
        for byte in message:
            print("\t Field: {:02X}".format(byte))

    def _parse_error_message(self, response):
        # Todo: on demand add error formats
        print("Received error response")
        if status == 0x02:
            pass
        elif status == 0x04:
            pass
        sys.exit(1)

    def read_settings(self, axis : Axis, measurement : Measurement):
        """
            Note:
                This requires message_identifiers to be disabled in CLS2Sim

            Todo:
                - move to documentation
                # COMMAND: Uint32 : 0xD0
                # Axis Uint32 : 0x01 elevator and 0x02 aileron
                # data id uint32: 0x10 axis position, 0x60 range

                - receive format
                # First 3 bytes are formated with
        """
        data_id = measurement2data_id[measurement]
        common_format = "HB"

        message = (Command.DataRead, axis2axis_id[axis], data_id)
        print('Send message:')
        self._print_message_fields(message)
        packet = struct.pack('<III', *message)
        print("Sent {} bytes".format(len(packet)))
        self.socket.sendto(packet, (self.host, self.port))

        # RECEIVE
        response, address = self.socket.recvfrom(self.buffer_size)
        print("Received: {} bytes (from {})".format(len(response), address))
        status = bytes([response[2]])
        print("Received status: {:02X}".format(status[0]))

        # Catch error in reading
        if not (status[0] == 0x00 ):
            self._parse_error_message(response)

        # Successful request status (expect device_id next)
        common_format += "H"
        format = common_format + measurement2format[measurement]
        message = struct.unpack('<' + format, response)
        print("Decoded message: ", message)
        print("--> [x] Received settings.")

    def _receive_threaded(self, shutdown_request):
        while not shutdown_request():
            try:
                self._receive()
            except socket.timeout as e:
                print('Error:\n', e)

    def _receive(self):
        try:
            response, address = self.socket.recvfrom(self.buffer_size)
            parsed = struct.unpack('<Iffffffff', response)
            if parsed[0] == 0xAE:
                self.state[:] = parsed[1:]
                # print('Parsed received bytes to : \n{}\n'.format(self.state))
        except socket.timeout as e:
            pass

    def launch_receiver_thread(self):
        thread = ShutdownCompliantThread(
            target=self._receive_threaded,
            shutdown_request=self._thread_shutdown_request)
        thread.start()
        self._receiver_thread_active = True
        
    def receive(self):
        self._receive()

    def get_state(self):
        return self.state
    
    def get_joystick_position_xy(self):
        return [self.state[0], self.state[2]]

    def actuate(self, x=0, y=0, safe=True):
        if safe and not input("Confirm with by pressing 'yes'") == "yes":
            print('Actuation aborted by user request')
            return
        self.force[0] = x
        self.force[2] = y
        packet = struct.pack('<Iiiiiiiii', 0xAE, *self.force.tolist())
        self.socket.sendto(packet, (self.host, self.port))
        print("Actuated: {}".format(self.force))

    def actuate_test(self, t):
        fmax = 180
        frequency = 0.3
        w = 2 * numpy.pi * frequency
        x = numpy.sin(w * t) * fmax
        y = numpy.cos(w * t) * fmax
        self.actuate(x, y, safe=False)


    def exit(self):
        self._thread_shutdown_request.set()
        self._receiver_thread_active = False

    @property
    def is_active(self):
        return self._receiver_thread_active


if __name__ == "__main__":
    print('Start module test')
    interface = Interface()

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

        interface.receive()

        t0 = time.time()
        t = 0
        while (t < 20) and interface.is_active:
            # interface.actuate(safe=False)
            interface.actuate_test(t)
            position = interface.get_state()
            print("{:.2f} | Position: {}".format(t, position))
            time.sleep(0.5)
            t = time.time() - t0
        print('--> [x] Completed module test')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)
    interface.exit()
