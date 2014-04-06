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

from traits.api import Instance, Dict, HasTraits, Array, Float, on_trait_change, List, Int, Button, Bool
from traitsui.api import Item, View, HGroup, VGroup, ArrayEditor, HSplit, TabularEditor
from traitsui.tabular_adapter import TabularAdapter

import struct
import math
import os
import numpy as np
import datetime

import sbp_piksi as ids


class SimulationSettings(HasTraits):
	"""
	SBP class for message SIMULATION_SETTINGS (0x0206)

	"""

	def __init__(self, d = None, l = None):
		if d is not None:
			self.from_binary(d)
		elif l is not None:
			self.from_list(l)
		else:
			self.center_ecef_x = 0.0
			self.center_ecef_y = 0.0
			self.center_ecef_z = 0.0
			self.speed = 0.0
			self.radius = 0.0
			self.pos_variance = 0.0
			self.speed_variance = 0.0
			self.starting_week_number = 0.0
			self.num_sats = 0.0
			self.mode = 0

	def from_binary(self, d):
		(
			self.center_ecef_x,
			self.center_ecef_y,
			self.center_ecef_z,
			self.speed,
			self.radius,
			self.pos_variance,
			self.speed_variance,
			self.starting_week_number,
			self.num_sats,
			self.mode,
		) = struct.unpack('<dddffffHBB', d)

	def to_binary(self):
		return struct.pack('<dddffffHBB',
			self.center_ecef_x,
			self.center_ecef_y,
			self.center_ecef_z,
			self.speed,
			self.radius,
			self.pos_variance,
			self.speed_variance,
			self.starting_week_number,
			self.num_sats,
			self.mode,
		)
	
	def to_list(self):
		l = [];
		l.append(['Simulation Mode', self.mode])
		l.append(['Speed (m/s)', self.speed])
		l.append(['Circle Radius (m)', self.radius])
		l.append(['Position Variance (m^2)', self.pos_variance])
		l.append(['Speed Variance (m^2)', self.speed_variance])
		l.append(['Centerpoint Coordinate X (ECEF)', self.center_ecef_x])
		l.append(['Centerpoint Coordinate Y (ECEF)', self.center_ecef_y])
		l.append(['Centerpoint Coordinate Z (ECEF)', self.center_ecef_z])
		l.append(['Simulated GPS Week Number', self.starting_week_number])
		l.append(['Simulated Number of Sate', self.num_sats])
		return l

	def from_list(self, l):
		self.mode = int(l[0][1]) % 256
		self.speed = float(l[1][1])
		self.radius = float(l[2][1])
		self.pos_variance = float(l[3][1])
		self.speed_variance = float(l[4][1])
		self.center_ecef_x = float(l[5][1])
		self.center_ecef_y = float(l[6][1])
		self.center_ecef_z = float(l[7][1])
		self.starting_week_number = int(l[8][1])
		self.num_sats = int(l[9][1])

	def __str__(self):
		return "%d %f %f %f %f %f %f %f %d %d" % (
			self.mode,
			self.speed,
			self.radius,
			self.pos_variance,
			self.speed_variance,
			self.center_ecef_x,
			self.center_ecef_y,
			self.center_ecef_z,
			self.starting_week_number,
			self.num_sats
		)


class SimpleAdapter(TabularAdapter):
	columns = [('Setting', 0), ('Value',  1)]

class SimulatorView(HasTraits):

	simulator_enable_button = Button(label='Enable hardware\'s simulation mode')
	simulator_disable_button = Button(label='Disable hardware\'s simulation mode')
	simulator_get_settings_button = Button(label='Get simulation settings from Piksi')
	simulator_set_settings_button = Button(label='Send simulation settings to Piksi')

	settings_table = List()

	traits_view = View(
		HSplit(
			Item('settings_table', editor = TabularEditor(adapter=SimpleAdapter()), show_label=False, width=0.6),
			VGroup(
				Item('simulator_enable_button', show_label=False),
			 	Item('simulator_disable_button', show_label=False),
				Item('simulator_get_settings_button', show_label=False),
			 	Item('simulator_set_settings_button', show_label=False)
			),
		)
	)

	##Simulator buttons
	def _simulator_enable_button_fired(self):
		print "Requesting piksi to enter simulation mode"
		data = struct.pack("<B", 1)
		self.link.send_message(ids.SIMULATION_MODE, data)

	def _simulator_disable_button_fired(self):
		print "Requesting piksi to enter simulation mode"
		data = struct.pack("<B", 0)
		self.link.send_message(ids.SIMULATION_MODE, data)

	def _simulator_get_settings_button_fired(self):
		print "Requesting simulation settings..."
		self.link.send_message(ids.SIMULATION_SETTINGS, '');

	def _simulator_set_settings_button_fired(self):
		print "Sending new simulation settings..."
		simulation_settings = SimulationSettings(l=self.settings_table)
		self.link.send_message(ids.SIMULATION_SETTINGS, simulation_settings.to_binary())

	##Callbacks for receiving messages

	def simulation_settings_message_callback(self, data):
		simulation_settings = SimulationSettings(d=data)
		self.settings_table = simulation_settings.to_list()

	def simulation_mode_message_callback(self, data):
		self.link.send_message(ids.SIMULATION_SETTINGS, '');
    
	def __init__(self, link):
		super(SimulatorView, self).__init__()

		self.link = link
		self.link.add_callback(ids.SIMULATION_SETTINGS, self.simulation_settings_message_callback)
		self.link.add_callback(ids.SIMULATION_MODE, self.simulation_mode_message_callback)

		#On startup, we request the current simulation mode and settings
		self.link.send_message(ids.SIMULATION_MODE, '');
		self.link.send_message(ids.SIMULATION_SETTINGS, '');

		self.python_console_cmds = {
			'sim': self
		}

