import os
import sys

sys.path.insert(0, os.path.abspath('.'))

project = 'point_cloud_transport'
copyright = '2023, Open Source Robotics Foundation, Inc.'
author = 'Open Source Robotics Foundation, Inc.'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.viewcode',
    'myst_parser',
]

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']

# Exclude root-level copies of user docs (rosdoc2 copies them to user_docs/ too).
exclude_patterns = [
    '_build',
    'overview.rst',
    'api.rst',
    'user_api.rst',
    'plugin_api.rst',
]

html_theme = 'sphinx_rtd_theme'

breathe_default_project = 'point_cloud_transport Doxygen Project'
