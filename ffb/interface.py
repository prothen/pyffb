#!/usr/bin/env python
"""


    Author: Philipp Rothenhäusler, Stockholm 2020

"""


import time
import attr
import yaml
import numpy
import typing
import socket
import struct


@attr.s
class Interface:
    # Variables
    force = attr.ib(default=None, type=typing.Optional[numpy.ndarray])
    position = attr.ib(default=None, type=typing.Optional[numpy.ndarray])

    # Configuration parameters
    config_name = attr.ib(default="default", type=str)
    configuration_path = attr.ib(default="../config", type=str)
    host = attr.ib(default=None, type=typing.Optional[str])
    port = attr.ib(default=None, type=typing.Optional[int])
    buffer_size = attr.ib(default=None, type=typing.Optional[int])
    timeout = attr.ib(default=None, type=typing.Optional[float])
    socket = attr.ib(default=None, type=typing.Optional[socket.socket])
    force_max = attr.ib(default=None, type=typing.Optional[float])

    def __attrs_post_init__(self):
        print('Custom initialisation after parsing')
        self._initialise_from_yaml()
        self._initialise_variables()

    def _initialise_from_yaml(self):
        config_name = self.config_name
        try:
            if config_name is not None:

                with open('{}/{}.yaml'.format(
                    self.configuration_path,
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
        self.force = numpy.zeros(8)
        self.position = numpy.zeros(8)

    def connect(self):
        print('Establish connection')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 5)
        sock.settimeout(self.timeout)
        self.socket = sock

    def receive(self):
        print('Launch threaded receiver')
        response, address = self.socket.recvfrom(self.buffer_size)
        parsed = struct.unpack('<Iffffffff', response)
        print('Parsed received bytes to : \n{}'.format(parsed))

    def states(self):
        print('Parse most recent received state')

    def actuate(self):
        request = struct.pack('<Iiiiiiiii', 0xAE, force[0], force[1], force[2], force[3], force[4], force[5], force[6], force[7]), (host, port)
        self.sock.sendto(request)


if __name__ == "__main__":
    print('Start module test')
    interface = Interface()
    print(interface)
    interface.connect()
    print('Now receive')
    interface.receive()
    print('--> [x] Completed module test')
