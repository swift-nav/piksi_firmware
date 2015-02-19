# -*- mode: python -*-

# Settings for PyInstaller

a = Analysis(['../console.py'],
             pathex=['../../libswiftnav/sbp_generate', '../'],
             hiddenimports = [
              'pyface.ui.qt4.init',
              'pyface.ui.qt4.action.menu_manager',
              'pyface.ui.qt4.action.menu_bar_manager',
              'pyface.ui.qt4.action.status_bar_manager',
              'pyface.ui.qt4.action.tool_bar_manager',
              'pyface.ui.qt4.resource_manager',
              'pyface.ui.qt4.clipboard',
              'pyface.ui.qt4.gui',
              'pyface.ui.qt4.image_resource',
              'pyface.ui.qt4.window',
              'pyface.ui.qt4.dialog',
              'pyface.ui.qt4.file_dialog',
              'pyface.ui.qt4.python_shell',
              'pyface.i_gui',
              'pyface.i_clipboard',
              'pyface.i_image_resource',
              'pyface.i_file_dialog',
              'pyface.i_dialog',
              'pyface.i_window',
              'pyface.i_python_shell',
              'pyface.qt.QtOpenGL',
              'enable.qt4.image',
              'enable.qt4.base_window',
              'enable.qt4.constants',
              'enable.toolkit_constants',
              'enable.qt4.scrollbar',
              'enable.savage.trait_defs.ui.qt4',
              'enable.savage.trait_defs.ui.qt4.svg_button_editor',
              'pyface.ui.qt4.python_editor',
              'pyface.i_python_editor',
             ],
             hookspath=None,
             runtime_hooks=['rthook_pyqt4.py'])

resources = [
  ('sbp_piksi.h', '../../src/sbp_piksi.h', 'DATA'),
  ('RELEASE-VERSION', '../RELEASE-VERSION', 'DATA'),
]
resources += Tree('../images', prefix='images')

import sys, os

kwargs = {}
exe_ext = ''
if os.name == 'nt':
  kwargs['icon'] = 'icon.ico'
  exe_ext = '.exe'
elif sys.platform.startswith('darwin'):
  kwargs['icon'] = 'icon.icns'

pyz = PYZ(a.pure)
exe = EXE(pyz,
          a.scripts,
          exclude_binaries=True,
          name='console'+exe_ext,
          debug=False,
          strip=None,
          upx=True,
          console=False,
          **kwargs
          )
coll = COLLECT(exe,
               resources,
               a.binaries,
               a.zipfiles,
               a.datas,
               strip=None,
               upx=True,
               name='console')

if sys.platform.startswith('darwin'):
  app = BUNDLE(coll,
               name='Piksi Console.app',
               icon='icon.icns')

