#!/usr/bin/env python
"""


    Author: Philipp Rothenhäusler, Stockholm 2020

"""

import os
import time
import attr
import yaml
import numpy
import typing
import socket
import struct
import signal
import threading


class ShutdownCompliantThread(threading.Thread):
    def __init__(self, shutdown_request, *args, **kwargs):
        self.shutdown_request : threading.Event = shutdown_request
        super().__init__(*args, **kwargs)

    def run(self):
        print('Thread #%s started' % self.ident)
        try:
            if self._target:
                self._target(*self._args, **self._kwargs, shutdown_request=self.shutdown_request.is_set)
        finally:
            del self._target, self._args, self._kwargs
        print('Thread #%s stopped' % self.ident)


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
        path = "../config/" if self.config_path is None else self.config_path
        cwd = os.path.dirname(os.path.abspath(__file__))
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
        self.state = numpy.zeros(8, dtype=numpy.int32)

    def connect(self):
        print('Establish connection')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 5)
        sock.settimeout(self.timeout)
        self.socket = sock

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
                print('Parsed received bytes to : \n{}'.format(parsed))
        except socket.timeout as e:
            pass

    def receive(self, threaded=True):
        if threaded:
            thread = ShutdownCompliantThread(target=self._receive_threaded, shutdown_request=self._thread_shutdown_request)
            thread.start()
            self._receiver_thread_active = True
        self._receive()

    def get_state(self):
        return self.state

    def actuate(self, safe=False):
        # print('Current force request is: \n{}'.format(self.force))
        if safe:
            if not input("Confirm with by pressing 'yes'") == "yes":
                print('Actuation aborted by user request')
                return
        packet = struct.pack('<Iiiiiiiii', 0xAE, *self.force.tolist())
        self.socket.sendto(packet, (self.host, self.port))

    def exit(self):
        self._thread_shutdown_request.set()
        self._receiver_thread_active = False

    @property
    def is_active(self):
        return self._receiver_thread_active


if __name__ == "__main__":
    print('Start module test')
    interface = Interface()

    try:
        interface.connect()
        print("Launch receiver callback")
        interface.receive()

        t0 = time.time()
        while (time.time() - t0 < 5) and interface.is_active:
            interface.actuate()
            position = interface.get_state()
            print("Position: ", position)
            time.sleep(0.5)
        print('--> [x] Completed module test')
    except Exception as e:
        print('--> [ ] Encountered exception: \n', e)
    interface.exit()
