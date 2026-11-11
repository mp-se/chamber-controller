# Sphinx configuration for Chamber Controller documentation.

project = 'Chamber Controller'
copyright = '2021-2026, Magnus Persson'
author = 'Magnus Persson'
release = '0.9.0'

extensions = [
    'sphinx_copybutton',
    'sphinx_design',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

html_theme = 'shibuya'
html_static_path = ['_static']
html_css_files = ['custom.css']
html_logo = 'images/chamber-controller-icon.svg'
html_favicon = 'images/chamber-controller-icon.svg'
