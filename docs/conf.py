#
# Sphinx documentation build configuration file
#

import os
import sys


sys.path.insert(0, os.path.abspath('..'))
#subprocess.call('doxygen Doxyfile.in', shell=True)
show_authors = True

project = 'Assignment 2'
copyright = '2025'
author = 'Arian Tavousi'
release = '0.1'

extensions = [
'sphinx.ext.autodoc',
'sphinx.ext.doctest',
'sphinx.ext.intersphinx',
'sphinx.ext.todo',
'sphinx.ext.coverage',
'sphinx.ext.mathjax',
'sphinx.ext.ifconfig',
'sphinx.ext.viewcode',
'sphinx.ext.githubpages',
"sphinx.ext.napoleon",
'sphinx.ext.inheritance_diagram',
'breathe'
]

templates_path = ['_templates']
exclude_patterns = []


html_theme = 'sphinx_rtd_theme'

autodoc_mock_imports = [
    "rospy",
    "std_srvs",
    "geometry_msgs",
    "nav_msgs",
    "actionlib",
    "tf",
    "sensor_msgs",
    "assignment_2_2024",
    "std_msgs",
    "actionlib_msgs"
]

intersphinx_mapping = {'python': ('https://docs.python.org/3', None)}

todo_include_todos = True

breathe_projects = {
"Assignment 2": "../build/xml/"
}
breathe_default_project = "Assignment 2"
breathe_default_members = ('members', 'undoc-members')



