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
              'pyface.ui.qt4.python_shell',
              'pyface.i_gui',
              'pyface.i_clipboard',
              'pyface.i_image_resource',
              'pyface.i_python_shell',
              'pyface.qt.QtOpenGL',
              'enable.qt4.image',
              'enable.qt4.base_window',
              'enable.qt4.constants',
              'enable.toolkit_constants',
              'enable.qt4.scrollbar',
              'enable.savage.trait_defs.ui.qt4',
              'enable.savage.trait_defs.ui.qt4.svg_button_editor',
              'pygments.lexers',
              'pygments.lexers._mapping',
              'pygments.lexers.compiled',
              'pygments.scanner',
              'pygments.lexers.functional',
              'pygments.lexers.jvm',
              'pygments.unistring',
              'pygments.lexers.agile',
              'pygments.styles',
              'pygments.styles.default',
              'pygments.style',
              'pyface.python_shell',
              'traitsui.qt4.shell_editor',
              'pyface.python_editor',
              'pyface.ui.qt4.python_editor',
              'pyface.i_python_editor',
              'pyface.ui.qt4.code_editor',
              'pyface.ui.qt4.code_editor.code_widget',
              'pyface.ui.qt4.code_editor.find_widget',
              'pyface.ui.qt4.code_editor.gutters',
              'pyface.ui.qt4.code_editor.replace_widget',
              'pyface.ui.qt4.code_editor.pygments_highlighter',
             ],
             hookspath=None,
             runtime_hooks=['rthook_pyqt4.py'])

resources = [
  ('sbp_piksi.h', '../../src/sbp_piksi.h', 'DATA'),
]
resources += Tree('../images', prefix='images')

pyz = PYZ(a.pure)
exe = EXE(pyz,
          a.scripts,
          exclude_binaries=True,
          name='console',
          debug=True,
          strip=None,
          upx=True,
          #console=True )
          console=False, icon='icon.icns')
coll = COLLECT(exe,
               resources,
               a.binaries,
               a.zipfiles,
               a.datas,
               strip=None,
               upx=True,
               name='console')
app = BUNDLE(coll,
             name='Piksi Console.app',
             icon='icon.icns')
