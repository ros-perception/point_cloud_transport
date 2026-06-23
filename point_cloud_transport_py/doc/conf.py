import contextlib
import inspect
import os
import re
import sys
import types

# Stub rpyutils.add_dll_directories_from_env as a no-op context manager.
# The real rpyutils may not be installed where rosdoc2 runs, and a plain
# MagicMock from autodoc_mock_imports does not support the `with` protocol
# that point_cloud_transport_py/__init__.py uses.
_rpyutils = types.ModuleType('rpyutils')


@contextlib.contextmanager
def _add_dll_directories_from_env(_env_var):
    yield


_rpyutils.add_dll_directories_from_env = _add_dll_directories_from_env
sys.modules.setdefault('rpyutils', _rpyutils)


def _install_stub_module(name, attrs):
    """Register a fake module with the given attributes in sys.modules.

    rosdoc2 overwrites ``autodoc_mock_imports`` with a list derived from
    ``exec_depends``, so a mock installed via that mechanism does not survive.
    Stubbing directly in ``sys.modules`` does survive and provides concrete
    attributes so ``from X import Y`` statements in the package __init__ work.
    """
    module = types.ModuleType(name)
    for attr in attrs:
        setattr(module, attr, type(attr, (), {}))
    sys.modules.setdefault(name, module)


_install_stub_module(
    'point_cloud_transport_py._point_cloud_transport',
    ['PointCloudTransport', 'Publisher', 'Subscriber'],
)
_install_stub_module(
    'point_cloud_transport_py._codec',
    ['PointCloudCodec', 'VectorString'],
)

# When rosdoc2 runs sphinx-build, it exec's this file from a generated wrapper.
# Parse the user_conf_py path out of the call stack and add the package root
# to sys.path so autodoc can import point_cloud_transport_py.
try:
    import point_cloud_transport_py  # noqa: F401
except ImportError:
    for _fi in inspect.stack():
        for _line in (_fi.code_context or []):
            _m = re.search(r'exec\(open\("([^"]+)"\)', _line)
            if _m:
                _pkg_root = os.path.dirname(os.path.dirname(_m.group(1)))
                if os.path.isdir(_pkg_root):
                    sys.path.insert(0, _pkg_root)
                break

project = 'point_cloud_transport_py'
copyright = '2023, Open Source Robotics Foundation, Inc.'
author = 'Open Source Robotics Foundation, Inc.'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.viewcode',
    'myst_parser',
]

# The pybind11 extension modules (_point_cloud_transport, _codec) are built by
# CMake and not importable during rosdoc2 builds.  Mock them so autodoc of the
# top-level package does not fail.
autodoc_mock_imports = [
    'point_cloud_transport_py._point_cloud_transport',
    'point_cloud_transport_py._codec',
    'rclpy',
    'sensor_msgs',
]

autodoc_member_order = 'bysource'

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']

exclude_patterns = [
    '_build',
    'overview.rst',
    'api.rst',
]

html_theme = 'sphinx_rtd_theme'
