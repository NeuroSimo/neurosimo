import sys
from pathlib import Path

project_root = Path(__file__).resolve().parents[2]
python_src_dir = str(project_root / "api/python")
sys.path.append(python_src_dir)

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'mtms'
copyright = '2023, connect2brain'
author = 'connect2brain'
release = '0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# Extensions
extensions = [
    # Generate documentation from docstrings
    'sphinx.ext.autodoc',
    # Generate summaries from docstrings
    'sphinx.ext.autosummary',
    # Map NumPy and Google docstring styles to reStructured text
    'sphinx.ext.napoleon',
    # Enable both .rst and .md files
    'myst_parser',
    # Enable Matlab autodoccing
    'sphinxcontrib.matlab'
]

exclude_patterns = ['_build']

## Include Python objects as they appear in source files
## Default: alphabetically ('alphabetical')
autodoc_member_order = 'bysource'

## Default flags used by autodoc directives
autodoc_default_options = {
    'members': True,
    'show-inheritance': True,
}

## Generate autodoc stubs with summaries from code
autosummary_generate = True

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'navigation_depth': 2
}
html_static_path:List = ['_static']

# -- Options for sphinxcontrib-matlabdomain ----------------------------------
# https://github.com/sphinx-contrib/matlabdomain#configuration 
matlab_src_dir = str(project_root / "api")
matlab_short_links = True
