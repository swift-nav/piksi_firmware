#!/usr/bin/env python

import yaml
import jinja2
import re
from settings_list import SettingsList
import os

swift_nav_style_path = "../libsbp/docs"
environment_variables_to_append=["TEXINPUTS","PATH"]
myenviron = os.environ
for each in environment_variables_to_append:
  try:
    texinputs = myenviron[each]
    print texinputs
    myenviron[each]=".:" + swift_nav_style_path + ":" + texinputs
  except KeyError:
    myenviron[each]=".:" + swift_nav_style_path
settings = SettingsList("settings.yaml")
groups = settings.return_groups()

#Note, these reg exps will not replace the '^' character to allow exponents in the units text field
LATEX_SUBS_ALLOW_EXPONENTS = (
    (re.compile(r'\\'), r'\\textbackslash'),
    (re.compile(r'([{}_#%&$])'), r'\\\1'),
    (re.compile(r'~'), r'\~{}'),
    (re.compile(r'_'), r'_'),
    (re.compile(r'"'), r"''"),
    (re.compile(r'\.\.\.+'), r'\\ldots'),
    (re.compile(r'\n'), r'\\newline ')
)

NO_UNDERSCORE = re.compile(r'_')

# We sometimes need to remove underscores.
# This will remove the latex safe underscore character and replace with a space
def no_us(value):
    newval = value
    try:
      return NO_UNDERSCORE.sub(' ', newval)
    except TypeError:
      pass
    return None

def escape_tex_exp(value):
    newval = value
    try:
      for pattern, replacement in LATEX_SUBS_ALLOW_EXPONENTS:
          newval = pattern.sub(replacement, newval)
      return newval
    except TypeError:
      pass
    return None

jenv = jinja2.Environment(
    block_start_string = '((*',
    block_end_string = '*))',
    variable_start_string = '(((',
    variable_end_string = ')))',
    comment_start_string = '((=',
    comment_end_string = '=))',
    loader=jinja2.FileSystemLoader("./")
)
jenv.filters['escape_tex_exp'] = escape_tex_exp
jenv.filters['no_us'] = no_us


latex_template = jenv.get_template('settings_template.tex')
with open("settings_out.tex", 'w') as f:
  f.write(latex_template.render(groups=sorted(groups), setting=sorted(settings.list_of_dicts), version='v0.15'))

import subprocess

subprocess.Popen(["pdflatex" , "--shell-escape", "settings_out.tex"], env=myenviron).wait()
subprocess.call(["mv" , "settings_out.pdf", "../docs/settings.pdf"])

