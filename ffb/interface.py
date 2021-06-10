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
    # Force max parameter (required)
    # without mount 900 - 3500
    force_max = attr.ib(default=None, type=typing.Optional[float])

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

    _debug_is_enabled = attr.ib(default=False, type=bool)

    def __attrs_post_init__(self):
        print('Custom initialisation after parsing')

        self._thread_shutdown_request = threading.Event()

        signal.signal(signal.SIGINT, self._exit_routine)
        signal.signal(signal.SIGTERM, self._exit_routine)

        self._initialise_from_yaml()
        if self.force_max is None:
            print('No force_max specified, set force_max=zero.')
        self._initialise_variables()

        self._debug("Force feedback TCP interface launched on:")
        self._debug("Host: ", self.host)
        self._debug("Port: ", self.port)

    def _debug(self, *args):
        if self._debug_is_enabled:
            print(*args)

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

    def _is_error(self, response):
        """ Common format based error decoding.

            Note:
                Expects uint16_t packet length and uint8_t error code.

        """
        # TODO: Check if equality bit based comparison is valid or & necessary
        status = response[2]
        if not status:
            print('Status is ok: ', status)
            return False
        elif status == 0x02:
            pass
        elif status == 0x04:
            pass
        print('Received error code in response: ', status)
        return True

    def update_setting(self, axis: Axis, command_type: CommandType, value):
        return self.send_command(
            command=Command().SettingsControl,
            axis=axis,
            command_type=command_type,
            value=value)

    def enable_override(self, axis: Axis, command_type):
        return self.send_command(
            command=Command().OverrideControl,
            axis=axis,
            command_type=command_type)

    def read_data(self, axis: Axis, command_type):
        return self.send_command(
            command=Command().DataRead,
            axis=axis,
            command_type=command_type)

    def send_command(self,
                     command: CommandDefinition,
                     command_type: CommandType,
                     axis: Axis = None,
                     profile=None,
                     value=None):
        """ Return raw response data for specified command type. """

        #self._debug("Send command: ", command, " ({})".format(command_type))

        message = (command.identifier,)
        request_format = command.request_format

        if command == Command.DataRead or \
                command == Command.OverrideControl:
            message += (axis, command_type.identifier)
        elif command == Command.SettingsControl:
            request_format += command_type.format
            message += (axis, command_type.identifier, value)

        ## TODO: Debug printing float message fields
        # self._print_message_fields(message)
        packet = struct.pack('<' + request_format, *message)
        self.socket.sendto(packet, (self.host, self.port))
        # print("Sent {} bytes".format(len(packet)))
        print('request_format: ', request_format)
        # Wait for command response
        response, address = self.socket.recvfrom(self.buffer_size)
        # print("Received: {} bytes (from {})".format(len(response), address))

        # Test if requested command was successful
        if self._is_error(response):
            print('Command processing failed. Are arguments correct?')
            raise RuntimeError()

        # In case of data read decode the received response and return
        if command.receives_value:
            response_format = command.response_format
            print('Response format: ', response_format)
            response_format += command_type.format
            print('Response format: ', response_format)
            data = struct.unpack('<' + response_format, response)
            print("Received data: ", data, " of type: ", type(data))
            print("--> [x] Received settings.")
            return data

    def _receive_threaded(self, shutdown_request):
        while not shutdown_request():
            try:
                self._receive()
            except socket.timeout as e:
                print('Error:\n', e)

    def _receive(self):
        """ Receive actuation response message and parse to position. """
        try:
            response, address = self.socket.recvfrom(self.buffer_size)
            parsed = struct.unpack('<Iffffffff', response)
            if parsed[0] == 0xAE:
                self.state[:] = parsed[1:]
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
        return [-2 * (self.state[2] - 0.5), 2 * (self.state[0] - .5)]

    def actuate(self, x=0, y=0, safe=True):
        if safe and not input("Confirm with by pressing 'yes'") == "yes":
            self._debug('Actuation aborted by user request')
            return
        ## Saturate input to range [-1, 1]
        x = max(-1, min(x, 1))
        y = max(-1, min(y, 1))

        print('APPLYING: ', self.force_max)
        ## Apply force to appropriate axis
        self.force[2] = x * self.force_max
        self.force[0] = - y * self.force_max
        packet = struct.pack('<Iiiiiiiii', 0xAE, *self.force.tolist())
        self.socket.sendto(packet, (self.host, self.port))
        # print("Actuated: {}".format(self.force))

    def actuate_test(self, t, stop=False):
        if stop:
            self.actuate(0, 0, safe=False)
            return
        fmax = 900
        frequency = .1
        w = 2 * numpy.pi * frequency
        x = numpy.sin(w * t) * fmax
        y = numpy.cos(w * t) * fmax
        self.actuate(x, y, safe=False)

    def exit(self):
        self.actuate_test(0, stop=True)
        self._thread_shutdown_request.set()
        self._receiver_thread_active = False

    @property
    def is_active(self):
        return self._receiver_thread_active
