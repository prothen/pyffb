#!/usr/bin/env python
"""
    Interface to Brunner CLS-E force feedback joystick.

    Todo:
        - add callback to receiver thread to for example forward to another interface


    Author: Philipp Rothenhäusler, Stockholm 2020

"""

import os
import attr
import time
import yaml
import socket
import struct
import signal
import threading


import ffb

from ffb.protocol import *


@attr.s
class Interface:

    # Flag indicating whether thread is alive
    _receiver_thread_active = attr.ib(default=False, type=bool)

    # Variables
    force = attr.ib(default=None, type=typing.Optional[numpy.ndarray])
    state = attr.ib(default=None, type=typing.Optional[numpy.ndarray])

    # Configuration parameters
    config_name = attr.ib(default="default", type=str)
    config_path = attr.ib(default=None, type=typing.Optional[str])
    host = attr.ib(default=None, type=typing.Optional[str])
    port = attr.ib(default=None, type=typing.Optional[int])
    buffer_size = attr.ib(default=None, type=typing.Optional[int])
    timeout = attr.ib(default=None, type=typing.Optional[float])
    socket = attr.ib(default=None, type=typing.Optional[socket.socket])

    # Maximal force applied through external force request
    # Valid range: 1 - 65535
    external_force_max = attr.ib(default=None, type=typing.Optional[float])

    # Autopilot active
    autopilot_active = attr.ib(default=True, type=bool)
    # Force applied by autopilot to reach position setpoint
    # Valid range: 1- 65535
    autopilot_force = attr.ib(default=None, type=typing.Optional[float])
    # Autopilot speed to reach position setpoint
    # Valid range: 1- 65535
    autopilot_speed = attr.ib(default=None, type=typing.Optional[float])

    # Measurement mapping using exponential based mapping
    exponential_scale = attr.ib(default=None,
                                type=typing.Optional[float])
    exponential_threshold = attr.ib(default=None,
                                    type=typing.Optional[float])

    # Minimum actuation frequency (>10Hz)
    frequency_minimum_actuation = attr.ib(default=10, type=float)
    # Actuation idle factor to apply transition force mapping
    _force_idle_factor: float = 0

    # Actuation time stamp to automatically disengage
    _timeperiod_next_action = attr.ib(default=None, type=typing.Optional[float])
    _deadline_next_actuation = attr.ib(default=None, type=typing.Optional[float])
    _actuation_idle_detected = attr.ib(default=False, type=bool)

    _debug_is_enabled = attr.ib(default=False, type=bool)

    def __attrs_post_init__(self):
        self._debug('Custom initialisation after parsing')

        self._thread_shutdown_request = threading.Event()

        signal.signal(signal.SIGINT, self._exit_routine)
        signal.signal(signal.SIGTERM, self._exit_routine)

        # Fetch configuration from local directory
        self._initialise_from_yaml()

        if self.external_force_max is None:
            self._debug('No force_max specified, set force_max=zero.')

        # Initialise variables
        self.force = numpy.zeros(8, dtype=numpy.int32)
        self.state = numpy.zeros(8, dtype=numpy.float32)
        self._timeperiod_next_action = 1. / self.frequency_minimum_actuation
        self._deadline_next_actuation = time.time() + self._timeperiod_next_action

        self._debug("Force feedback TCP interface launched on:")
        self._debug("Host: {} and Port: {} ".format(self.host, self.port))

    def _debug(self, *args):
        if self._debug_is_enabled:
            self._debug('Pyffb: ', *args)

    def _exit_routine(self, sig, frame):
        """ Gracefully disconnect """
        self._debug('Caught shutdown interrupt request!')
        self.exit()

    def _initialise_from_yaml(self):
        """ Parse YAML configuration and combine with arguments. """
        config_name = self.config_name
        cwd = os.path.dirname(os.path.abspath(__file__))
        path = "{}/../config".format(cwd) if self.config_path is None else self.config_path
        self._debug("Pyunity3d - Configurator: Current absolute path: ", cwd)
        try:
            if config_name is not None:

                with open('{}/{}.yaml'.format(
                        path,
                        config_name), 'r') as stream:
                    try:
                        config = yaml.safe_load(stream)
                    except yaml.YAMLError as e:
                        self._debug('Configuration parsing failed due to:\n', e)
                # Argument compliant selection of default config parameters
                config_current = self.__dict__
                for key, value in config.items():
                    if key not in config_current:
                        config_current.update({key: value})
        except Exception as e:
            self._debug('Pyunity3d: Configuration loading failed due to:\n', e)
            raise RuntimeError()
        self._debug('--> [x] Configuration parsed successfully.')
        self._debug('--> Loaded:\n {}'.format(config_name))

    def _debug_message_fields(self, message):
        for byte in message:
            self._debug("\t Field: {:02X}".format(byte))

    def _is_error(self, response):
        """ Common format based error decoding.

            Note:
                Expects uint16_t packet length and uint8_t error code,
                which means 3 bytes (e.g. 0x0ABCDE)

        """
        status = response[2]
        if not status:
            self._debug('Status is ok: ', status)
            return False
        elif status == 0x02:
            pass
        elif status == 0x04:
            pass
        self._debug('Received error code in response: ', status)
        return True

    def _is_actuation_stream_active(self):
        """ Returns true if actuation idle is detected.

            Note:
                Once detection happens variables
                for transition are initialised.

        """

        if self._deadline_next_actuation < time.time():
            if not self._actuation_idle_detected:
                # Disengage routine and communication
                print('Autopilot actuation idle detected!')
                print('--> Disengage by removing applied force.')
                self._actuation_idle_detected = True
                self._force_idle_factor = 0
                self.set_autopilot_force(axis=Axis.X, force=0)
                self.set_autopilot_force(axis=Axis.Y, force=0)
            return True
        elif self._actuation_idle_detected:
            # Re-engage routine and communication
            print('Autopilot actuation stream regained!')
            print('--> Re-enable force actuation. ')
            self._actuation_idle_detected = False
            self._force_idle_factor = 1
            self.set_autopilot_force(axis=Axis.X, force=self.autopilot_force)
            self.set_autopilot_force(axis=Axis.Y, force=self.autopilot_force)
        return False

    def _receive_threaded(self, shutdown_request):
        while not shutdown_request():
            try:
                stamp = time.time()
                self._receive()
                # TODO: Remove printouts for frequency
                print('Reception frequency: ',
                      1./(time.time() - stamp))
                # Test if actuation
                self._is_actuation_stream_active()
            except socket.timeout as e:
                self._debug('Error:\n', e)

    def _receive(self):
        """ Receive actuation response message and parse to position. """
        try:
            response, address = self.socket.recvfrom(self.buffer_size)
            parsed = struct.unpack('<Iffffffff', response)
            if parsed[0] == 0xAE:
                self.state[:] = parsed[1:]
        except socket.timeout as e:
            pass

    def _exponential_mapping(self, value):
        """ Return rescaled value based on exponential mapping. """
        magnitude = max(self.exponential_threshold - numpy.abs(value), 0)
        return 1 / numpy.exp(self.exponential_scale * magnitude) * value

    def _post_process_exponential(self, vector: numpy.ndarray):
        """ Return post processed vector using exponential mapping. """
        # Todo: Debug mapping and add post processing matplotlib curve
        return numpy.apply_along_axis(self._exponential_mapping,
                                      vector,
                                      axis=0)

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

        self._debug("Send command: ", command, " ({})".format(command_type))

        message = (command.identifier,)
        request_format = command.request_format

        if command == Command.DataRead or \
                command == Command.OverrideControl:
            message += (axis, command_type.identifier)
        elif command == Command.SettingsControl:
            request_format += command_type.format
            message += (axis, command_type.identifier, value)

        # Prepare message
        packet = struct.pack('<' + request_format, *message)
        self.socket.sendto(packet, (self.host, self.port))
        self._deadline_next_actuation = time.time() + self._timeperiod_next_action
        self._debug('request_format: ', request_format)

        # Wait for command response
        response, address = self.socket.recvfrom(self.buffer_size)

        # Test if requested command was successful
        if self._is_error(response):
            # TODO: Alternatively return False for external error handling
            self._debug('Command processing failed. Are arguments correct?')
            raise RuntimeError()

        # In case of data read decode the received response and return
        if command.receives_value:
            response_format = command.response_format
            response_format += command_type.format
            data = struct.unpack('<' + response_format, response)
            self._debug("Received data: ", data, " of type: ", type(data))
            self._debug("--> [x] Received settings.")
            return data

    def _activate_autopilot_on_axis(self, axis : Axis):
        """ Enable autopilot and corresponding overrides on axis. """

        self._debug(axis, ': Set autopilot speed...')
        self.update_setting(
            axis=axis,
            command_type=SettingsControl.AutopilotSpeed,
            value=self.autopilot_speed)

        self._debug(axis, ': Set autopilot force...')
        self.update_setting(
            axis=axis,
            command_type=SettingsControl.AutopilotForce,
            value=self.autopilot_force)

        self._debug(axis, ': Set autopilot initial position...')
        self.update_setting(
            axis=axis,
            command_type=SettingsControl.AutopilotPosition,
            value=numpy.array(0))

        self._debug(axis, ': Enable autopilot...')
        self.update_setting(
            axis=axis,
            command_type=SettingsControl.AutopilotEnable,
            value=1)

        self._debug(axis, ': Enable autopilot overwrite...')
        self.enable_override(
            axis=axis,
            command_type=OverrideControl.AutopilotEnableOverride)

        self._debug(axis, ': Enable autopilot position override...')
        self.enable_override(
            axis=axis,
            command_type=OverrideControl.AutopilotPositionOverride)

    def activate_autopilot(self):
        """ Activate autopilot for x and y axis based interaction. """
        self._activate_autopilot_on_axis(Axis.X)
        self._activate_autopilot_on_axis(Axis.Y)

    def connect(self):
        """ Connect to CLS2SIM interface. """
        self._debug('Connecting...')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(self.timeout)
        self.socket = sock
        self._debug('--> [x] Connected')

    def launch_receiver_thread(self):
        """ Launch a non-blocking receiver thread fetching data. """
        thread = ffb.custom_thread.ShutdownCompliantThread(
            target=self._receive_threaded,
            shutdown_request=self._thread_shutdown_request)
        thread.start()
        self._receiver_thread_active = True

    def exit(self):
        """ Process exit request to end interface gracefully. """
        self.actuate(x=0, y=0)
        self._thread_shutdown_request.set()
        self._receiver_thread_active = False

    def receive(self):
        """ Return a single position measurement. """
        return self._receive()

    def set_autopilot_force(self, axis: Axis, force: float):
        """ Set autopilot force for axis. """
        self.update_setting(
            axis=axis,
            command_type=SettingsControl.AutopilotForce,
            value=numpy.array(force))

    def get_axes_xy(self):
        """ Return 2 dimensional vector for xy axis position measurement. """
        return numpy.array([
            -2 * (self.state[2] - 0.5),
            2 * (self.state[0] - .5)])

    def get_axes_xy_processed(self):
        """ Return exponential post processed xy axis measurement. """
        p = self.get_axes_xy()
        return self._post_process_exponential(p)

    def set_position(self, x=0, y=0):
        """ Set setpoint for autopilot to xy position. """

        # Saturate input to range [-1, 1]
        x = max(-1, min(x, 1))
        y = max(-1, min(y, 1))

        self.update_setting(
            axis=Axis.X,
            command_type=SettingsControl.AutopilotPosition,
            value=numpy.array(x))

        self.update_setting(
            axis=Axis.Y,
            command_type=SettingsControl.AutopilotPosition,
            value=numpy.array(y))

    def actuate(self, x=0, y=0):
        """ Actuate external force command to joystick. """

        # Saturate input to range [-1, 1]
        x = max(-1, min(x, 1))
        y = max(-1, min(y, 1))

        self._debug('External force applied: ', self.external_force_max.flatten())

        # Apply force to appropriate axis
        self.force[2] = x * self.external_force_max * self._force_idle_factor
        self.force[0] = - y * self.external_force_max * self._force_idle_factor
        packet = struct.pack('<Iiiiiiiii', 0xAE, *self.force.tolist())
        self.socket.sendto(packet, (self.host, self.port))

    def actuate_safe(self, safe=True, *args, **kwargs):
        """ Safe actuation for test setups to confirm single actuation. """
        if safe and not input("Confirm with by pressing 'yes'") == "yes":
            self._debug('Actuation aborted by user request')
            return
        return self.actuate(*args, **kwargs)

    def actuate_test(self, t, stop=False):
        """ Time varying force actuation reference script. """
        if stop:
            self.actuate(x=0, y=0)
            return
        fmax = 900
        frequency = .1
        w = 2 * numpy.pi * frequency
        x = numpy.sin(w * t) * fmax
        y = numpy.cos(w * t) * fmax

        self.actuate(x=x, y=y)

    @property
    def is_active(self):
        return self._receiver_thread_active
