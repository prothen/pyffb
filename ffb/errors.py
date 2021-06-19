#!/usr/bin/env python


__author__ = "Philipp Rothenhäusler"
__version__ = "1.0"
__status__ = "Development"
__copyright__ = "Copyright 2021 Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"


class ConfigurationParametersNotLoadedError(Exception):
    """ Indicate error if configuration is not found or loaded incomplete. """
    pass
