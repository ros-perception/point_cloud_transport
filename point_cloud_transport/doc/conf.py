# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

sys.path.insert(0, os.path.abspath('.'))

project = 'point_cloud_transport'
project_copyright = '2023, Open Source Robotics Foundation, Inc.'
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
