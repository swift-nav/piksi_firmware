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

import struct
import threading
import time
import sys

import cPickle as pickle
import calendar
import json

from sbp          import crc16, SBP, SBP_PREAMBLE
from sbp.piksi    import SBP_MSG_PRINT, SBP_MSG_RESET
from sbp.standard import SBP_MSG_HEARTBEAT

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 1000000

class ListenerThread (threading.Thread):

  def __init__(self, link, print_unhandled=False):
    super(ListenerThread, self).__init__()
    self.link = link
    self.wants_to_stop = False
    self.print_unhandled = print_unhandled
    self.daemon = True

  def stop(self):
    self.wants_to_stop = True

  def run(self):
    while not self.wants_to_stop:
      try:
        m = self.link.get_message()
        mt = m.msg_type
        # Will throw away last message here even if it is valid.
        if self.wants_to_stop:
          if self.link.ser:
            self.link.ser.close()
          break
        if mt is not None:
          for cb in self.link.get_global_callbacks():
            cb(m)
          cbs = self.link.get_callback(mt)
          if cbs is None or len(cbs) == 0:
            if self.print_unhandled:
              print "Host Side Unhandled message %02X" % mt
          else:
            for cb in cbs:
              cb(m)
      except (IOError, OSError):
        # Piksi was disconnected
        print "ERROR: Piksi disconnected!"
        return
      except:
        import traceback
        print traceback.format_exc()

def list_ports(self=None):
  import serial.tools.list_ports
  ports = serial.tools.list_ports.comports()
  # Remove ports matching "ttyS*" (non-virtual serial ports on Linux).
  ports = filter(lambda x: x[1][0:4] != "ttyS", ports)
  if not any(ports):
    return None
  else:
    return ports

class SerialLink:

  def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, use_ftdi=False, print_unhandled=False):
    self.print_unhandled = print_unhandled
    self.unhandled_bytes = 0
    self.callbacks = {}
    self.global_callbacks = []
    if use_ftdi:
      import pylibftdi
      self.ser = pylibftdi.Device()
      self.ser.baudrate = baud
    else:
      import serial
      try:
        self.ser = serial.Serial(port, baud, timeout=1)
      except serial.SerialException:
        print
        print "Serial device '%s' not found" % port
        print
        print "The following serial devices were detected:"
        print
        for p in list_ports():
          p_name, p_desc, _ = p
          if p_desc == p_name:
            print "\t%s" % p_name
          else:
            print "\t%s (%s)" % (p_name, p_desc)
        sys.exit(1)

    # Delay then flush the buffer to make sure the receive buffer starts empty.
    time.sleep(0.5)
    self.ser.flush()

    self.lt = ListenerThread(self, print_unhandled)
    self.lt.start()

  def __del__(self):
    self.close()

  def close(self):
    try:
      self.lt.stop()
      while self.lt.isAlive():
        time.sleep(0.1)
    except AttributeError:
      pass

  def get_message(self):
    while True:
      if self.lt.wants_to_stop:
        return SBP(None, None, None, None, None)

      # Sync with magic start bytes
      magic = self.ser.read(1)
      if magic:
        if ord(magic) == SBP_PREAMBLE:
          break
        else:
          self.unhandled_bytes += 1
          if self.print_unhandled:
            print "Host Side Unhandled byte : 0x%02x," % ord(magic), \
                                                            "total", \
                                               self.unhandled_bytes

    hdr = ""
    while len(hdr) < 5:
      hdr = self.ser.read(5 - len(hdr))

    msg_type, sender_id, msg_len = struct.unpack('<HHB', hdr)
    crc = crc16(hdr, 0)

    data = ""
    while len(data) < msg_len:
      data += self.ser.read(msg_len - len(data))
    crc = crc16(data, crc)

    crc_received = ""
    while len(crc_received) < 2:
      crc_received = self.ser.read(2 - len(crc_received))
    crc_received = struct.unpack('<H', crc_received)[0]

    if crc != crc_received:
      print "Host Side CRC mismatch: 0x%04X 0x%04X" % (crc, crc_received)
      return SBP(None, None, None, None, None)

    return SBP(msg_type, sender_id, msg_len, data, crc)

  def send_message(self, msg_type, msg, sender_id=0x42):
    framed_msg = struct.pack('<BHHB', SBP_PREAMBLE, msg_type, sender_id, len(msg))
    framed_msg += msg
    crc = crc16(framed_msg[1:], 0)
    framed_msg += struct.pack('<H', crc)

    self.ser.write(framed_msg)

  def send_char(self, char):
    self.ser.write(char)

  def add_callback(self, msg_type, callback):
    """
    Add a named callback for a specific SBP message type.
    """
    try:
      self.callbacks[msg_type].append(callback)
    except KeyError:
      self.callbacks[msg_type] = [callback]

  def add_global_callback(self, callback):
    """
    Add a global callback for all SBP messages.
    """
    self.global_callbacks.append(callback)

  def rm_callback(self, msg_type, callback):
    try:
      self.callbacks[msg_type].remove(callback)
    except KeyError:
      print "Can't remove callback for msg 0x%04x: message not registered" \
            % msg_type
    except ValueError:
      print "Can't remove callback for msg 0x%04x: callback not registered" \
            % msg_type

  def get_callback(self, msg_type):
    """
    Retrieve a named callback for a specific SBP message type.
    """
    if msg_type in self.callbacks:
      return self.callbacks[msg_type]
    else:
      return None

  def get_global_callbacks(self):
    """
    Retrieve a named callback for a specific SBP message type, or a global
    callback for all SBP messages.
    """
    return self.global_callbacks

  def wait_message(self, msg_type, timeout=None):
    ev = threading.Event()
    d = {'data': None}
    def cb(sbp_msg):
      d['data'] = sbp_msg.payload
      ev.set()
    self.add_callback(msg_type, cb)
    ev.wait(timeout)
    self.rm_callback(msg_type, cb)
    return d['data']

def default_print_callback(sbp_msg):
  sys.stdout.write(sbp_msg.payload)

def default_log_callback(handle):
  """
  Callback for binary serializing Python objects to a file with a
  consistent logging format:

    (delta, timestamp, item) : tuple
      delta = msec reference timestamp (int)
      timestamp = current timestamp (int - UTC epoch)
      item = Python object to serialize

  Parameters
  ----------
  handle : file
    An already-opened file handle

  Returns
  ----------
  pickler : lambda data
    Function that will serialize Python object to open file_handle

  """
  ref_time = time.time()
  protocol = 2
  return lambda data: pickle.dump(format_log_entry(ref_time, data),
                                  handle,
                                  protocol)

def default_log_json_callback(handle):
  """
  Callback for JSON serializing Python objects to a file with a
  consistent logging format:

    {'delta': delta, 'timestamp': timestamp, 'data': data} : dict
      delta = msec reference timestamp (int)
      timestamp = current timestamp (int - UTC epoch)
      data = Python object to JSON serialize

  Parameters
  ----------
  handle : file
    An already-opened file handle

  Returns
  ----------
  pickler : lambda data
    Function that will JSON serialize Python object to open file_handle

  """
  ref_time = time.time()
  return lambda data: handle.write(json.dumps(format_log_json_entry(ref_time, data)) + '\n')

def generate_log_filename():
  """
  Generates a consistent filename for logging, for example:
  serial_link_log_20141125-140552.log

  Returns
  ----------
  filename : str

  """
  return time.strftime("serial_link_log_%Y%m%d-%H%M%S.log")

def format_log_entry(t0, item):
  """
  Generates a Python object with logging metadata.

  Parameters
  ----------
  t0 : float
    Reference time (seconds)
  item : object
    Python object to serialize

  Returns
  ----------
  (delta, timestamp, item) : tuple
    delta = msec reference timestamp (int)
    timestamp = current timestamp (int - UTC epoch)
    item = Python object to serialize
  """
  timestamp = calendar.timegm(time.gmtime())
  delta = int((time.time() - t0)*1000)
  return (delta, timestamp, item)

def format_log_json_entry(t0, item):
  """
  Generates a Python object with logging metadata.

  Parameters
  ----------
  t0 : float
    Reference time (seconds)
  item : object
    Python object to serialize

  Returns
  ----------
  {'delta': delta, 'timestamp': timestamp, 'data': data} : dict
    delta = msec reference timestamp (int)
    timestamp = current timestamp (int - UTC epoch)
    data = Python object to JSON serialize
  """
  timestamp = calendar.timegm(time.gmtime())
  delta = int((time.time() - t0)*1000)
  data = item.to_json_dict()
  return {'delta': delta, 'timestamp': timestamp, 'data': data}

class Watchdog:
  """
  Watchdog wraps a timer with a callback that can rearm the timer.

  Parameters
  ----------
  timeout : float
    timeout of timer in seconds
  alarm : callback
    function to call when/if timer expires
  """
  def __init__(self, timeout, alarm):
    self.timeout = timeout
    self.alarm = alarm
    self.timer = None

  def __call__(self, *args):
    self.call()

  def call(self):
    """
    Rearm the timer.
    """
    if self.timer:
      self.timer.cancel()
    self.timer = threading.Timer(self.timeout, self.alarm)
    self.timer.daemon = True
    self.timer.start()

def default_watchdog_alarm():
  """
  Called when the watchdog timer alarms. Will raise a KeyboardInterrupt to the
  main thread and exit the process.
  """
  sys.stderr.write("ERROR: Watchdog expired!")
  import thread
  thread.interrupt_main()

if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(description='Swift Nav Serial Link.')
  parser.add_argument('-p', '--port',
                      default=[DEFAULT_PORT],
                      nargs=1,
                      help='specify the serial port to use.')
  parser.add_argument("-b", "--baud",
                      default=[DEFAULT_BAUD], nargs=1,
                      help="specify the baud rate to use.")
  parser.add_argument("-v", "--verbose",
                      help="print extra debugging information.",
                      action="store_true")
  parser.add_argument("-f", "--ftdi",
                      help="use pylibftdi instead of pyserial.",
                      action="store_true")
  parser.add_argument("-l", "--log",
                      action="store_true",
                      help="serialize SBP messages to autogenerated log file.")
  parser.add_argument("-j", "--json",
                      action="store_true",
                      help="JSON serialize SBP messages.")
  parser.add_argument("-w", "--watchdog",
                      default=[None], nargs=1,
                      help="alarm after WATCHDOG seconds have elapsed without heartbeat.")
  parser.add_argument("-t", "--timeout",
                      default=[None], nargs=1,
                      help="exit after TIMEOUT seconds have elapsed.")
  parser.add_argument("-r", "--reset",
                      action="store_true",
                      help="reset device after connection.")
  args = parser.parse_args()
  serial_port = args.port[0]
  baud = args.baud[0]
  link = SerialLink(serial_port, baud, use_ftdi=args.ftdi,
                    print_unhandled=args.verbose)
  link.add_callback(SBP_MSG_PRINT, default_print_callback)
  # Setup logging
  log_file = None
  if args.log:
    log_name = generate_log_filename()
    log_file = open(log_name, 'w+')
    print "Logging at %s." % log_name
    if args.json:
      link.add_global_callback(default_log_json_callback(log_file))
    else:
      link.add_global_callback(default_log_callback(log_file))
    if args.reset:
      link.send_message(SBP_MSG_RESET, '')
  # Setup watchdog
  watchdog = args.watchdog[0]
  if watchdog:
    link.add_callback(SBP_MSG_HEARTBEAT, Watchdog(float(watchdog), default_watchdog_alarm))
  try:
    if args.timeout[0] is None:
      # Wait forever until the user presses Ctrl-C
      while True:
        time.sleep(0.1)
    else:
      # Wait until the timeout has elapsed
      expire = time.time() + float(args.timeout[0])
      while time.time() < expire:
        time.sleep(0.1)
      sys.stdout.write("Timer expired!\n")
  except KeyboardInterrupt:
    # Callbacks, such as the watchdog timer on SBP_HEARTBEAT call
    # thread.interrupt_main(), which throw a KeyboardInterrupt
    # exception. To get the proper error condition, return exit code
    # of 1. Note that the finally block does get caught since exit
    # itself throws a SystemExit exception.
    sys.exit(1)
  finally:
    if log_file:
      log_file.close()
    link.close()
