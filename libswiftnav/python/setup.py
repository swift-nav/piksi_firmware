# Build script for 'swiftnav' - Python libswiftnav bindings

# Change this as needed
libswiftnav_path = ".."

import sys, os, stat, commands
from distutils.core import setup
from distutils.extension import Extension

# We'd better have Cython installed, or it's a no-go
try:
  from Cython.Distutils import build_ext
except:
  print "You don't seem to have Cython installed. Please get a"
  print "copy from www.cython.org and install it"
  sys.exit(1)

# Scan a directory for extension files, converting them to extension names in
# dotted notation
def scan_dir(dir, files=[]):
  for file in os.listdir(dir):
    path = os.path.join(dir, file)
    if os.path.isfile(path) and path.endswith(".pyx"):
      files.append(path.replace(os.path.sep, ".")[:-4])
    elif os.path.isdir(path):
      scan_dir(path, files)
  return files

# Generate an Extension object from its dotted name
def make_extension(ext_name):
  ext_path = ext_name.replace(".", os.path.sep) + ".pyx"
  return Extension(
    ext_name,
    [ext_path],
    include_dirs = [libswiftnav_path, "."],
    library_dirs = [libswiftnav_path],
    extra_compile_args = ["-O3", "-Wall"],
    extra_link_args = ['-g'],
    libraries = ["m", "swiftnav-x86"],
  )

# get the list of extensions
ext_names = scan_dir("swiftnav")

# and build up the set of Extension objects
extensions = [make_extension(name) for name in ext_names]

# finally, we can pass all this to distutils
setup(
  name = "swiftnav",
  packages = ["swiftnav"],
  ext_modules = extensions,
  cmdclass = {'build_ext': build_ext},
)
