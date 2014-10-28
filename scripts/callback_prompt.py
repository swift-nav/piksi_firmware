#!/usr/bin/env python
# Copyright (C) 2014 Swift Navigation Inc.
# Contact: Colin Beighley <colin@swift-nav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

from traits.api import HasTraits, Event, String, Button, Instance
from traitsui.api import View, Handler, Action, Item, TextEditor, VGroup, UItem
from pyface.api import GUI

from threading import Thread
from time import sleep

update_button = Action(name = "Update", action = "set_execute_callback_true", \
                             show_label=False)
reset_button = Action(name = "Reset", action = "set_execute_callback_true", \
                             show_label=False)
close_button = Action(name = "Close", action = "set_execute_callback_false", \
                             show_label=False)

from new import instancemethod

import sys, os
from pyface.image_resource import ImageResource
if getattr(sys, 'frozen', False):
    # we are running in a |PyInstaller| bundle
    basedir = sys._MEIPASS
    os.chdir(basedir)
else:
    # we are running in a normal Python environment
    basedir = os.path.dirname(__file__)
icon = ImageResource('icon',
         search_path=['images', os.path.join(basedir, 'images')])

# Handler methods that can be associated with buttons.
def set_execute_callback_true(self, info):
  info.object.execute_callback = True
  info.object.handler_executed = True

def set_execute_callback_false(self, info):
  info.object.execute_callback = False
  info.object.handler_executed = True

class CallbackHandler(Handler):

  # Register action handlers from passed list.
  def __init__(self, actions):
    super(CallbackHandler, self).__init__()
    for a in actions:
      # Add instancemethod to self for Action.action.
      handler = globals()[a.action]
      self.__dict__[a.action] = instancemethod(handler, self, CallbackHandler)

  # X button was pressed.
  def close(self, info, is_ok):
    info.object.handler_executed = True
    info.object.closed = True
    return True

  # Handles parent object's close Event being written to.
  def object_close_changed(self, info):
    info.object.closed = True
    info.ui.owner.close()

class CallbackPrompt(HasTraits):

  text = String
  close = Event

  def __init__(self, title, actions, callback=None):
    self.callback = callback

    self.handler_executed = False
    self.execute_callback = False
    self.closed = False
    self.close = 0

    self.view = View(
      Item(
        'text',
        style='readonly',
        editor=TextEditor(),
        show_label=False
      ),
      buttons=actions,
      title=title,
      handler=CallbackHandler(actions),
      icon = icon,
      resizable=True,
    )

  def run(self, block=True):
    try:
      if self.thread.is_alive():
        return
    except AttributeError:
      pass

    self.thread = Thread(target=self._run)
    self.thread.start()

    if block:
      self.wait()

  def _run(self):
    GUI.invoke_later(self.edit_traits, self.view)
    while not self.handler_executed:
      sleep(0.1)

    if self.execute_callback:
      GUI.invoke_later(self.callback)

    if not self.closed:
      self.close = 1

  def wait(self):
    while not self.closed:
      sleep(0.1)

  def kill(self):
    self.handler_executed = True
    if not self.closed:
      self.close = 1

