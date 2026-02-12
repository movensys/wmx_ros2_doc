# -- Project information -----------------------------------------------------
project = 'WMX ROS2 Documentation'
copyright = '2026, Movensys'
author = 'Movensys'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'myst_parser',           # Markdown support
    'sphinx.ext.autodoc',    # Auto-generate from docstrings
    'sphinx.ext.intersphinx',# Cross-reference other Sphinx docs
    'sphinx.ext.todo',       # TODO directives
]

# Markdown support
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_theme_options = {
    'logo_only': False,
    'navigation_depth': 3,
    'collapse_navigation': False,
}

# -- Intersphinx mapping (link to ROS2 docs) --------------------------------
intersphinx_mapping = {
    'ros2': ('https://docs.ros.org/en/humble/', None),
}
