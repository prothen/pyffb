#!/usr/bin/env python
""" Customised threading class to terminate under Windows. """


__author__ = "Philipp Rothenhäusler"
__version__ = "1.0"
__status__ = "Development"
__copyright__ = "Copyright 2021 Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"


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
