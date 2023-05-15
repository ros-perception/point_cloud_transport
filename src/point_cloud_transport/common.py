# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Common definitions."""

from ctypes import RTLD_GLOBAL
import importlib

from cras.ctypes_utils import load_library
from cras.string_utils import STRING_TYPE
from dynamic_reconfigure.msg import Config, BoolParameter, DoubleParameter, IntParameter, StrParameter


__library = None


def _get_base_library():
    global __library
    if __library is None:
        __library = load_library('point_cloud_transport', mode=RTLD_GLOBAL)
        if __library is None:
            return None

    return __library


def _dict_to_config(d):
    """Convert configuration dict to :class:`dynamic_reconfigure.msg.Config`.

    :param d: Configuration dict (or already the message, in which case it is just returned).
    :type d: dict or dynamic_reconfigure.msg.Config or None
    :return: The config message.
    :rtype: dynamic_reconfigure.msg.Config
    """
    if d is None:
        return Config()
    if isinstance(d, Config):
        return d
    c = Config()
    for key, value in d.items():
        if isinstance(value, bool):
            c.bools.append(BoolParameter(key, value))
        elif isinstance(value, float):
            c.doubles.append(DoubleParameter(key, value))
        elif isinstance(value, int):
            c.ints.append(IntParameter(key, value))
        elif isinstance(value, STRING_TYPE):
            c.strs.append(StrParameter(key, value))
    return c


def _c_array(data, c_type):
    c_data = (c_type * (len(data) + 1))()
    c_data[:-1] = data
    c_data[-1] = 0
    return c_data


class _TransportInfo(object):
    def __init__(self, name, topic, data_type, config_data_type=None):
        self.name = name
        self.topic = topic
        self.data_type = data_type
        self.config_data_type = config_data_type


__pkg_modules = dict()


def get_cfg_type(cfg_type_str):
    """Load and return a Python module corresponding to the given ROS dynamic reconfigure type.

    :param str cfg_type_str: Type of the config in textual form (e.g. `compressed_image_transport/CompressedPublisher`).
    :return: The Python module.
    """
    if len(cfg_type_str) == 0:
        return None
    pkg, cfg = cfg_type_str.split('/')
    cfg += 'Config'
    cfg_module = pkg + '.cfg.' + cfg
    if cfg_module not in __pkg_modules:
        __pkg_modules[cfg_module] = importlib.import_module(cfg_module)
    return __pkg_modules[cfg_module]
