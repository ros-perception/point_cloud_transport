# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Common definitions."""

from ctypes import RTLD_GLOBAL

from cras.ctypes_utils import load_library


__library = None


def _get_base_library():
    global __library
    if __library is None:
        __library = load_library('point_cloud_transport', mode=RTLD_GLOBAL)
        if __library is None:
            return None

    return __library


class _TransportInfo(object):
    def __init__(self, name, topic, data_type, config_data_type=None):
        self.name = name
        self.topic = topic
        self.data_type = data_type
        self.config_data_type = config_data_type
