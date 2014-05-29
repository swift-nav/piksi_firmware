#!/usr/bin/env python
# Copyright (C) 2011-2014 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""Contains the class OutputStream, a HasTraits file-like text buffer."""

from traits.api import HasTraits, Str, Bool, Trait, Int
from traitsui.api import View, UItem, TextEditor, Handler
from traits.etsconfig.api import ETSConfig
from pyface.api import GUI


DEFAULT_MAX_LEN = 8000


class _OutputStreamViewHandler(Handler):

    def object_text_changed(self, uiinfo):
        ui = uiinfo.ui
        if ui is None:
            return

        for ed in  ui._editors:
            if ed.name == 'text':
                break
        else:
            # Did not find an editor with the name 'text'.
            return

        if ETSConfig.toolkit == 'wx':
            # With wx, the control is a TextCtrl instance.
            ed.control.SetInsertionPointEnd()
        elif ETSConfig.toolkit == 'qt4':
            # With qt4, the control is a QtGui.QTextEdit instance.
            from pyface.qt.QtGui import QTextCursor
            ed.control.moveCursor(QTextCursor.End)


class OutputStream(HasTraits):
    """This class has methods to emulate an file-like output string buffer.

    It has a default View that shows a multiline TextEditor.  The view will
    automatically move to the end of the displayed text when data is written
    using the write() method.

    The `max_len` attribute specifies the maximum number of bytes saved by
    the object.  `max_len` may be set to None.

    The `paused` attribute is a bool; when True, text written to the
    OutputStream is saved in a separate buffer, and the display (if there is
    one) does not update.  When `paused` returns is set to False, the data is
    copied from the paused buffer to the main text string.
    """

    # The text that has been written with the 'write' method.
    text = Str

    # The maximum allowed length of self.text (and self._paused_buffer).
    max_len = Trait(DEFAULT_MAX_LEN, None, Int)

    # When True, the 'write' method appends its value to self._paused_buffer
    # instead of self.text.  When the value changes from True to False,
    # self._paused_buffer is copied back to self.text.
    paused = Bool(False)

    # String that holds text written while self.paused is True.
    _paused_buffer = Str

    def write(self, s):
        if self.paused:
            self._paused_buffer = self._truncated_concat(self._paused_buffer, s)
        else:
            self.text = self._truncated_concat(self.text, s)

    def flush(self):
        GUI.process_events()

    def close(self):
        pass

    def reset(self):
        self._paused_buffer = ''
        self.paused = False
        self.text = ''

    def _truncated_concat(self, text, s):
        if len(s) >= self.max_len:
            # s could be huge. Handle this case separately, to avoid the temporary
            # created by 'text + s'.
            result = s[-self.max_len:]
        else:
            result = (text + s)[-self.max_len:]
        return result

    def _paused_changed(self):
        if self.paused:
            # Copy the current text to _paused_buffer.  While the OutputStream
            # is paused, the write() method will append its argument to _paused_buffer.
            self._paused_buffer = self.text
        else:
            # No longer paused, so copy the _paused_buffer to the displayed text, and
            # reset _paused_buffer.
            self.text = self._paused_buffer
            self._paused_buffer = ''

    def traits_view(self):
        view = \
            View(
                UItem('text', editor=TextEditor(multi_line=True), style='custom'),
                handler=_OutputStreamViewHandler(),
            )
        return view

