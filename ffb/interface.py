#!/usr/bin/env python
""" Interface to Brunner CLS-E force feedback joystick. """

__author__ = "Philipp Rothenhäusler"
__version__ = "1.0"
__status__ = "Development"
__copyright__ = "Copyright 2021 Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"


import os
import sys
import attr
import time
import yaml
import socket
import struct
import signal
import random
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

    # Direct argument for configuration parameters that superseed
    # both default parameters and config_name + config_path
    _config = attr.ib(default=None, type=typing.Optional[dict])

    # Network configuration
    host = attr.ib(default=None, type=typing.Optional[str])
    port = attr.ib(default=None, type=typing.Optional[int])
    buffer_size = attr.ib(default=None, type=typing.Optional[int])
    timeout = attr.ib(default=None, type=typing.Optional[float])
    socket = attr.ib(default=None, type=typing.Optional[socket.socket])

    # Maximal force applied through external force request
    # Valid range: 1 - 65535
    external_force_max = attr.ib(default=None, type=typing.Optional[float])

    # Autopilot active
    _autopilot_activated = attr.ib(default=True, type=bool)
    # Force applied by autopilot to reach position setpoint
    # Valid range: 1 - 65535
    autopilot_force = attr.ib(default=None, type=typing.Optional[float])
    # Autopilot speed to reach position setpoint
    # Valid range: 5 - 9999
    autopilot_speed = attr.ib(default=None, type=typing.Optional[float])

    # Measurement mapping using exponential based mapping
    exponential_scale = attr.ib(default=None,
                                type=typing.Optional[float])
    exponential_threshold = attr.ib(default=None,
                                    type=typing.Optional[float])

    # Actuation idle factor to apply transition force mapping
    _force_idle_factor: float = 0

    # Flag to indicate idle detection state transitioning
    _actuation_idle_detected = attr.ib(default=False, type=bool)

    # Desired actuation frequency (>10Hz)
    frequency_desired_actuation = attr.ib(default=None,
                                          type=typing.Optional[float])

    # Minimum actuation frequency (>10Hz)
    frequency_minimum_actuation = attr.ib(default=None,
                                          type=typing.Optional[float])

    # Time period according to desired actuation frequency
    _timeperiod_next_actuation = attr.ib(
        default=None, type=typing.Optional[float])

    # Time period according to minimal actuation frequency
    _time_period_next_actuation_maximum = attr.ib(
        default=None, type=typing.Optional[float])

    # Time stamp for frequency determination of measurement reception
    _stamp_axes_state = attr.ib(default=None, type=typing.Optional[float])
    _stamp_last_actuation = attr.ib(default=None, type=typing.Optional[float])

    _debug_is_enabled = attr.ib(default=False, type=bool)

    has_new_message = attr.ib(default=False, type=bool)

    statistics = attr.ib(default=None, type=typing.Optional[dict])

    def __attrs_post_init__(self):
        self._debug('Custom initialisation after parsing')

        self._thread_shutdown_request = threading.Event()

        # Initialise statistics
        s = dict()
        s['sent_commands'] = 0
        s['received_commands'] = 0
        s['external_actuation_messages'] = 0
        self.statistics = s

        # Implement signal handling in wrapping class to call exit() to cleanup
        # Todo: Required under Windows (for disengagement exit_routine)
        # signal.signal(signal.SIGINT, self._exit_routine)
        # signal.signal(signal.SIGTERM, self._exit_routine)

        # Fetch configuration from local directory
        self._initialise_from_yaml()

        if self.external_force_max is None:
            self._debug('No force_max specified, set force_max=zero.')

        # Initialise variables
        self.force = numpy.zeros(8, dtype=numpy.int32)
        self.state = numpy.zeros(8, dtype=numpy.float32)

        # Initialise timing conditions
        self._time_period_next_actuation = \
            self.frequency_desired_actuation ** -1

        self._time_period_next_actuation_maximum = \
            self.frequency_minimum_actuation ** -1

        # Initialise time stamp axes state measurement reception
        self._stamp_axes_state = time.time()
        # Initialise time stamp for last actuation
        self._stamp_last_actuation = time.time()

        self._debug("Force feedback TCP interface launched on:")
        self._debug("Host: {} and Port: {} ".format(self.host, self.port))
        self._debug("-> Ready to execute connect()")

    def _debug(self, *args):
        if self._debug_is_enabled:
            print('Pyffb: ', *args)

    def _exit_routine(self, sig, frame):
        """ Gracefully disconnect """
        self._debug('Caught shutdown interrupt request!')
        self.exit()
        # Pass signal interrupt to cascading processes
        os.kill(os.getpid(), signal.SIGINT)

    def _initialise_from_yaml(self):
        """ Parse YAML configuration and combine with arguments. """
        config_name = self.config_name
        # Use as default the files absolute working directory
        cwd = os.path.dirname(os.path.abspath(__file__))
        if self.config_path is None:
            path = "{}/../config".format(cwd)
        else:
            path = self.config_path

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
                    if key not in config_current or config_current[key] is None:
                        config_current.update({key: value})
        except Exception as e:
            self._debug('Pyunity3d: Configuration loading failed due to:\n', e)
            raise ffb.errors.ConfigurationParametersNotLoadedError()

        # Overwrite configuration with any provided dictionary config
        print('self._config: ', self._config)
        if self._config is not None:
            self.__dict__.update(self._config)

        self._debug('--> [x] Configuration parsed successfully.')
        self._debug('--> Loaded:\n {}'.format(config_name))

    def _measure_reception_frequency(self):
        """ Update frequency based on first order system. """
        print('Reception frequency: ',
              1. / (time.time() - self._stamp_axes_state))

        self._stamp_axes_state = time.time()

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
            return False
        elif status == 0x02:
            pass
        elif status == 0x04:
            pass
        self._debug('Received error code in response: ', status)
        return True

    def _is_actuation_idle(self):
        """ Return true if last actuation is older than maximally admitted. """
        deadline = self._stamp_last_actuation + \
                   self._time_period_next_actuation_maximum
        return deadline < time.time()

    def actuation_stream_is_active(self):
        """ Returns true if actuation is active.

            Note:
                Once detection happens variables
                for transitioning are initialised.

        """

        if self._is_actuation_idle():
            if not self._actuation_idle_detected:
                # Disengage routine and communication
                print('Actuation idle detected!')
                print('--> Disengage by removing applied force.')

                self._actuation_idle_detected = True
                self._force_idle_factor = 0

                self.actuate(x=0, y=0, now=True)
                self.deactivate_autopilot()
            return False
        elif self._actuation_idle_detected:
            # Re-engage routine and communication
            print('Actuation stream regained!')
            print('--> Re-enable force actuation. ')

            self._actuation_idle_detected = False
            if self._autopilot_activated:
                self.activate_autopilot()
        self._force_idle_factor = 1
        return True

    def _receive(self):
        """ Receive actuation response message and parse to position. """
        try:
            ## TODO: Nonblocking recvfrom!
            response, address = self.socket.recvfrom(self.buffer_size)
            if response[0] == 0xAE:
                parsed = struct.unpack('<Iffffffff', response)
                self.state[:] = parsed[1:]
                self.has_new_message = True
                self._measure_reception_frequency()
        except (socket.timeout, Exception) as e:
            pass

    def _receive_threaded(self, shutdown_request):
        """ Runs non-blocking loop for reception and idle detection. """
        try:
            while not shutdown_request():
                try:
                    self._receive()
                    self.actuation_stream_is_active()
                except socket.timeout as e:
                    self._debug('Error:\n', e)
        except Exception as e:
            sys.exit(0)

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


        s = self.statistics

        # self._debug("Send command: ", command.identifier)

        message = (command.identifier,)
        request_format = command.request_format

        if command == Command.DataRead or \
                command == Command.OverrideControl:
            message += (axis, command_type.identifier)
        elif command == Command.SettingsControl:
            request_format += command_type.format
            message += (axis, command_type.identifier, value)

        # If external_message_identifier is enabled add id at front
        # TODO: Find randomised message id
        # TODO: Check that id is non-zero
        message_id = random.randint(1, 4294967296)

        request_format = "I" + request_format
        message = (message_id,) + message

        # Prepare message
        packet = struct.pack('<' + request_format, *message)
        self.socket.sendto(packet, (self.host, self.port))
        s['sent_commands'] += 1

        try:
            response_format = "HIB"
            # Expect response within 5ms
            time_to_wait = .01
            expiration_deadline = time.time() + time_to_wait

            # Message Id expected as
            message_id_received = 0x00

            while expiration_deadline > time.time() and \
                not message_id == message_id_received:
                try:
                    response, address = self.socket.recvfrom(self.buffer_size)
                    # Slice first 7 bytes
                    data = struct.unpack('<' + response_format, response[:7])
                    length = data[0]
                    message_id_received = data[1]
                    error_code = data[2]
                    # messaged_id_received = data[1] # struct.unpack('<' + response_format, response)
                    # message_id_received = response[1]
                except (socket.timeout, Exception) as e:
                    pass

            # Either expiration for waiting reached or message id is correct
            if not message_id == message_id_received:
                return None
            # Otherwise delete message identifier and continue as usual
            response = response[:2] + response[6:]


            # Test if requested command was successful
            if self._is_error(response):
                # TODO: Alternatively return False for external error handling
                self._debug('Command processing failed. Are arguments correct?')
                raise RuntimeError()

            # Any non-erronous message is seen as successful reception
            s['received_commands'] += 1

            # In case of data read decode the received response and return
            if command.receives_value:
                response_format = command.response_format
                response_format += command_type.format
                data = struct.unpack('<' + response_format, response)
                # self._debug("Received data: ", data, " of type: ", type(data))
                return data
        except Exception as e:
            print('Error due to : ', e)
            return None

    def _activate_autopilot_on_axis(self, axis: Axis):
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

        self._switch_autopilot_mode(axis, enable=True)

        self._debug(axis, ': Enable autopilot overwrite...')
        self.enable_override(
            axis=axis,
            command_type=OverrideControl.AutopilotEnableOverride)

        self._debug(axis, ': Enable autopilot position override...')
        self.enable_override(
            axis=axis,
            command_type=OverrideControl.AutopilotPositionOverride)

    def initialise_autopilot(self):
        """ Activate autopilot for x and y axis based interaction. """
        self._activate_autopilot_on_axis(Axis.X)
        self._activate_autopilot_on_axis(Axis.Y)

    def _switch_autopilot_mode(self, axis: Axis, enable: bool):
        self._debug(axis, ': Autopilot status: ', enable)
        self.update_setting(
            axis=axis,
            command_type=SettingsControl.AutopilotEnable,
            value=enable)

    def deactivate_autopilot(self):
        """ Deactivate autopilot on both axes. """

        self._debug('Disable autopilot overrides...')
        self.enable_override(
            axis=0x00,
            command_type=OverrideControl.AutopilotEnableOverride)
        self.enable_override(
            axis=0x00,
            command_type=OverrideControl.AutopilotPositionOverride)

        self._switch_autopilot_mode(Axis.X, enable=False)
        self._switch_autopilot_mode(Axis.Y, enable=False)

    def activate_autopilot(self):
        """ Activate autopilot for x and y axis based interaction. """
        # Re-enable overrides and initial configuration and setpoints
        self.initialise_autopilot()

    def enable_autopilot(self, enable: bool):
        self._autopilot_activated = enable
        if enable:
            self.activate_autopilot()
            return
        # Disable autopilot engagement
        self.deactivate_autopilot()

    def connect(self):
        """ Connect to CLS2SIM interface. """
        self._debug('Connecting...')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(.01)
        #sock.setblocking(0)
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
        self._debug("Execute exit routine to cleanup interface...")
        self.actuate(x=0, y=0, now=True)
        self.deactivate_autopilot()
        self._thread_shutdown_request.set()
        self._receiver_thread_active = False

    def receive(self):
        """ Return a single axes state measurement. """
        return self._receive()

    def set_autopilot_force(self, axis: Axis, force: float):
        """ Set autopilot force for axis. """
        self.update_setting(
            axis=axis,
            command_type=SettingsControl.AutopilotForce,
            value=numpy.array(force))

    def reset_statistics(self):
        """ Reset the interface statistics. """
        s = self.statistics
        s['sent_commands'] = 0
        s['received_commands'] = 0
        s['external_actuation_messages'] = 0

    def get_axes_xy(self, mark_fetched=True):
        """ Return 2 dimensional vector for xy axis measurement. """
        self.has_new_message = False
        return numpy.array([
            -2 * (self.state[2] - 0.5),
            2 * (self.state[0] - .5)])

    def get_axes_xy_processed(self):
        """ Return exponential post processed xy axis measurement. """
        p = self.get_axes_xy()
        return self._post_process_exponential(p)

    def set_position(self, x=0, y=0):
        """ Set setpoint for autopilot to xy position. """
        if not self._is_time_to_actuate():
            return

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

    def _get_position(self, axis: Axis):
        x = self.read_data(
            axis=axis,
            command_type=DataRead.PositionNormalized)

        return x[-1] if x is not None else None

    def get_position(self):
        """ Return joystick position in 2 dimensional vector in ENU frame. """
        x = self._get_position(Axis.X)
        y = self._get_position(Axis.Y)

        if x is not None and y is not None:
            return numpy.array([x, y])
        return None

    def _is_time_to_actuate(self):
        """ Return true if deadline for next actuation is reached. """

        if self._stamp_last_actuation + self._time_period_next_actuation < time.time():
            dt = time.time() - self._stamp_last_actuation
            self._stamp_last_actuation = time.time()
            return True
        return False

    def actuate(self, x, y, now=False):
        """ Actuate external force command to joystick.

            Note:
                Expects (x,y) applied in ENU coordinate frame
                according to REP105 and converts it internally to
                corresponding aileron and elevator.

        """
        if not now and not self._is_time_to_actuate():
            return

        # Saturate input to range [-1, 1]
        x = max(-1, min(x, 1))
        y = max(-1, min(y, 1))

        # Apply force to appropriate axis
        # x: - elevator
        # y: - aileron

        self.force[0] = - x * self.external_force_max * self._force_idle_factor
        self.force[2] = - y * self.external_force_max * self._force_idle_factor

        packet = struct.pack('<IIiiiiiiii', 0, 0xAE, *(self.force.tolist()))
        self.socket.sendto(packet, (self.host, self.port))
        self.statistics['external_actuation_messages'] += 1

    def actuate_safe(self, safe=True, *args, **kwargs):
        """ Safe actuation for test setups to confirm single actuation. """
        if safe and not input("Confirm with by pressing 'yes'") == "yes":
            self._debug('Actuation aborted by user request')
            return
        return self.actuate(*args, **kwargs)

    def actuate_test(self, t, stop=False):
        """ Time varying force actuation reference script. """
        if stop:
            self.actuate(x=0, y=0, now=True)
            return

        frequency = .1
        w = 2 * numpy.pi * frequency
        x = numpy.sin(w * t)
        y = numpy.cos(w * t)

        self.actuate(x=x, y=y)

    @property
    def is_active(self):
        return self._receiver_thread_active
