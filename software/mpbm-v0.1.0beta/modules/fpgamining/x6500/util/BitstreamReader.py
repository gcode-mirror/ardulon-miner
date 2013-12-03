# Copyright (C) 2011 by fpgaminer <fpgaminer@bitcoin-mining.com>
#                       fizzisist <fizzisist@fpgamining.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# Parse a .BIT file generated by Xilinx's bitgen.
# That is the default file generated during ISE's compilation.
#
# FILE FORMAT:
#
# Consists of an initial 11 bytes of unknown content (???)
# Then 5 fields of the format:
#  1 byte key
#  2 byte, Big Endian Length (EXCEPT: The last field, which has a 4 byte length)
#  data (of length specified above ^)
# 
# The 5 fields have keys in the sequence a, b, c, d, e
# The data from the first 4 fields are strings:
# design name, part name, date, time
# The last field is the raw bitstream.
#

import time
import struct

# Dictionary for looking up idcodes from device names:
idcode_lut = {'6slx150fgg484': 0x401d093, '6slx45csg324': 0x4008093, '6slx150tfgg676': 0x403D093}

class BitFileReadError(Exception):
  _corruptFileMessage = "Unable to parse .bit file; header is malformed. Is it really a Xilinx .bit file?"

  def __init__(self, value=None):
    self.parameter = BitFileReadError._corruptFileMessage if value is None else value
  def __str__(self):
    return repr(self.parameter)
    
class BitFileMismatch(Exception):
  _mismatchMessage = "Device IDCode does not match bitfile IDCode! Was this bitstream built for this FPGA?"

  def __init__(self, value=None):
    self.parameter = BitFileReadError._mismatchMessage if value is None else value
  def __str__(self):
    return repr(self.parameter)
    
class BitFileUnknown(Exception):
  _unknownMessage = "Bitfile has an unknown UserID! Was this bitstream built for the X6500?"
  def __init__(self, value=None):
    self.parameter = BitFileReadError._unknownMessage if value is None else value
  def __str__(self):
    return repr(self.parameter)
  
class Object(object):
  pass

class BitFile:
  """Read a .bit file and return a BitFile object."""
  @staticmethod
  def read(name):
    with open(name, 'rb') as f:
      bitfile = BitFile()
      
      # 11 bytes of unknown data
      if BitFile._readLength(f) != 9:
        raise BitFileReadError()
      
      BitFile._readOrDie(f, 11)
      
      bitfile.designname = BitFile._readField(f, b"a").decode("latin1").rstrip('\0')
      bitfile.userid = int(bitfile.designname.split(';')[-1].split('=')[-1], base=16)
      bitfile.part = BitFile._readField(f, b"b").decode("latin1").rstrip('\0')
      bitfile.date = BitFile._readField(f, b"c").decode("latin1").rstrip('\0')
      bitfile.time = BitFile._readField(f, b"d").decode("latin1").rstrip('\0')
      bitfile.idcode = idcode_lut[bitfile.part]
      
      if bitfile.userid == 0xFFFFFFFF:
        bitfile.rev = 0
        bitfile.build = 0
      elif (bitfile.userid >> 16) & 0xFFFF == 0x4224:
        bitfile.rev = (bitfile.userid >> 8) & 0xFF
        bitfile.build = bitfile.userid & 0xFF
      else:
        raise BitFileUnknown()
      
      if BitFile._readOrDie(f, 1) != b"e":
        raise BitFileReadError()
      
      length = BitFile._readLength4(f)
      bitfile.bitstream = BitFile._readOrDie(f, length)
      
      return bitfile
  
  # Read a 2-byte, unsigned, Big Endian length.
  @staticmethod
  def _readLength(filestream):
    return struct.unpack(">H", BitFile._readOrDie(filestream, 2))[0]

  @staticmethod
  def _readLength4(filestream):
    return struct.unpack(">I", BitFile._readOrDie(filestream, 4))[0]

  # Read length bytes, or throw an exception
  @staticmethod
  def _readOrDie(filestream, length):
    data = filestream.read(length)

    if len(data) < length:
      raise BitFileReadError()

    return data

  @staticmethod
  def _readField(filestream, key):
    if BitFile._readOrDie(filestream, 1) != key:
      raise BitFileReadError()

    length = BitFile._readLength(filestream)
    data = BitFile._readOrDie(filestream, length)

    return data
    

  def __init__(self):
    self.designname = None
    self.rev = None
    self.build = None
    self.part = None
    self.date = None
    self.time = None
    self.length = None
    self.idcode = None
    self.bitstream = None

