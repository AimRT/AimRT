# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'AimRT'
copyright = "2025, Agibot"
author = 'Agibot'
release = "v1.0.0"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx_multiversion",
    "myst_parser",
    "sphinx_design",
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# smv_tag_whitelist = r"^v\d+\.\d+\.\d+$"
smv_tag_whitelist = r"^(v1\.1\.0|v1\.0\.0|v0\.10\.0|v0\.9\.3|v0\.8\.3)$"

# Whitelist pattern for branches (set to None to ignore all branches)
smv_branch_whitelist = None

# Whitelist pattern for remotes (set to None to use local branches only)
smv_remote_whitelist = None

# Pattern for released versions
smv_released_pattern = r"^.*$"

# Format for versioned output directories inside the build directory
smv_outputdir_format = "{ref.name}"

# Determines whether remote or local git branches/tags are preferred if their output dirs conflict
smv_prefer_remote_refs = False


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']

html_theme = 'sphinx_rtd_theme'

html_theme_options = {
    "display_version": True,
}

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

myst_enable_extensions = [
    "dollarmath",
    "amsmath",
    "deflist",
    "html_admonition",
    "html_image",
    "colon_fence",
    "smartquotes",
    "replacements",
    "linkify",
    "substitution",
    "tasklist",
]

myst_substitutions = {
    "code_site_url": "https://github.com/AimRT/AimRT",
    "code_site_root_path_url": "https://github.com/AimRT/AimRT/blob/main",
}

html_show_sourcelink = False

html_css_files = [
    "css/custom.css",
]

html_context = {
    "current_version": "v1.0.0",
    "versions": [
        {"name": "latest", "url": "./latest/"},
        {"name": "v0.8.3", "url": "./v0.8.3/"},
        {"name": "v0.9.3", "url": "./v0.9.3/"},
        {"name": "v0.10.0", "url": "./v0.10.0/"},
        {"name": "v1.0.0", "url": "./v1.0.0/"},
    ],
}
