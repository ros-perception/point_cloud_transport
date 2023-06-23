# Configuration file for the Sphinx documentation builder.
import os
os.environ['CRAS_DOCS_COMMON_SPHINX_PACKAGE_PATH'] = \
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
from cras_docs_common.sphinx_docs_conf import *  # noqa: E402, I201, F401, F403
