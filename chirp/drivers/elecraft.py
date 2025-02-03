""" Attempt at driver for Elecraft K- and KX-Series HF radios,
    pretending the Elecraft is a clone-mode radio rather than live.
    sync_in and sync_out access all memories via serial port.
    Adapted from template.py
"""
# Copyright 2024 Declan Rieb <wd5eqy@arrl.net>
# Copyright 2012 Dan Smith <dsmith@danplanet.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import logging
import os
import string
import time
from struct import unpack, calcsize
import threading
import serial
import struct
from typing import NamedTuple
from chirp import chirp_common, memmap
from chirp import bitwise
from chirp import directory
from chirp import errors
from chirp import settings

LOG = logging.getLogger(__name__)

# Here is where we define the memory map for the radio. Since
# frequency (in Hertz) and an eight-character alpha tag
#

MEM_FORMAT = """
struct elecraft_freq {
    u8  megahz;
    u8  tenkhz;         // 0-0x63=99
    u8  hundredhz;      // 0-0x63=99
    u8  tenhz;          // 0-9
    u8  hz;             // 0-9
    };

struct memslot {
struct elecraft_freq vfoa;
struct elecraft_freq vfob;
    u8  modeb:4,
        modea:4;
    u8  digimode:4,     // Submode if either mode is DATA
        b75:1,          // 0-> 45/31 Baud, 1-> 75 Baud
        unk:3;
    u8  ant:1,          // 0-> Antenna 1, 1-> Antenna 2
        rev:1,          // Reverse mode (CW-R, DATA-R, AM-S)
        nb:1,           // noise blanker?
        unk0:1,
        pre:1,          // Preamplifier?
        unk01:2,
        att:1;          // Attenuator
    u8  rxant:1,        // Receive antenna
        flag3:7;
    u8  flag4:7,
        split:1;        // Split mode (TX A, RX B)
    u8  unk1:3,
        xv:1,           // Xverter in use
        band:4;         // 0-0xA (normal bands) 0-8 (XV1-XV9)
    };

struct extra {
    u8  subtone;
    u8  offset;         // 00-0xFA, 20kHz/step
    u8  unk3:5,         // Repeater flags
        tone:1,         // Tone off/on
        minus:1,        // 0 -> shift+  1 -> shift-
        duplex:1;       // simplex/duplex
    u32 flags;
    u8  empty[9];
    u8  label[5];       // Displayed with memory... NOT ASCII
    u8  comment[24];
    };

// This is not really how Elecraft stores memory, 
//  but jibes with the way the driver reads it.
struct {
    struct memslot regular_memory;
    struct extra extra_memory;
    i32 cheksum;        // Calculation done on last byte, but sign extended
} radiomem[100];
struct memslot normal_bands[24]; // 11-15 future, 16-24 Xvtr

struct {
    u16 state[5];
} transverter_state[9]; // unknown, but probably contain mixer frequency

"""

class ElecraftMem(NamedTuple):
    """ Elecraft regular  memory format (unpacked from 0x40 bytes) """
    address: int
    length: int
    rvfoa: bytes
    rvfob: bytes
    modes: bytes
    dmode: bytes
    f2: bytes
    f3: bytes
    f4: bytes
    band: bytes
    subtone: bytes
    offset: bytes
    repflag: bytes
    f9: bytes
    fa: bytes
    fb: bytes
    fc: bytes
    fd: bytes
    rlabel: bytes
    comment: bytes
    cksum: int

""" Magic for decoding the radio's responses are in FMT here:
    Assumes 68 characters, converted from text hex string of 136 chars
    These will be decoded into the ElecraftMem NamedTuple above
    >   Big-endian
    H   unsigned short (2 bytes):  address
    B   Unsigned char-sized integer: length
    5s  string, freq spec (two, one per VFO)
    B   unsigned char-sized integer: Modes
    13B for other flags
    x   5 ignored
    5s  string, label (in vc format)
    24s string, comment field (in ascii)
    i   4-byte signed integer for checksum
"""
FMT = '>HB5s5sB13B8x5s24si'

# All Elecraft commands driver uses:
READ_CMD = "ER"     # MC on K4
WRIT_CMD = "EW"     # MC on K4
IDEN_CMD = "ID"
XTRA_CMD = "OM"
AUTO_CMD = "AI"

MEM_START = 0x0C00
MEM_LEN = 0X40

MODES = {0: "CW", 1: "USB", 2: "LSB", 3: "DN", 4: "AM", 5: "FM"}

# Elecraft radios (<K4?) don't use ASCII
VALID_CHARS = ' ' + string.ascii_uppercase + string.digits + '*+/@_'
XTABLE = {}
REVTABLE = {}
for index in range(0, len(VALID_CHARS)):
    XTABLE[index] = VALID_CHARS[index]
    REVTABLE[VALID_CHARS[index]] = index

RADIO_IDS = {       # ID always returns 017, except for K4
                    # periods mean "i don't care"
    "ID017": "Elecraft",
    "ID017X": "K3",     # OM returns "..........--"
    "ID0171": "KX2",    # OM returns "..........01"
    "ID0172": "KX3",    # OM returns "..........02"
    "ID0174": "K4"      # OM returns "........4---"
}

LOCK = threading.Lock()
COMMAND_RESP_BUFSIZE = 200
LAST_BAUD = 38400
LAST_DELIMITER = (";", "")

# The Elecraft radios use ";"
# as a CAT command message delimiter, and all others use "\n".

def _command(port: serial.serialposix.Serial, cmd: str, *args) -> str:
    """ _command sends the "cmd" with "args" over serial port "ser"
        AND THEN reads and returns the response from same port. """
    # Send @cmd to radio via @ser
    global LOCK, LAST_DELIMITER, COMMAND_RESP_BUFSIZE

    start = time.time()

    # yet to do: This global use of LAST_DELIMITER breaks reentrancy
    # and needs to be fixed.
    if args:
        cmd += LAST_DELIMITER[1] + LAST_DELIMITER[1].join(args)
    cmd += LAST_DELIMITER[0]

    LOG.debug("PC->RADIO: %s" % cmd.strip())
    port.write(cmd.encode('cp1252'))

    result = ""
    while not result.endswith(LAST_DELIMITER[0]):
        result += port.read(COMMAND_RESP_BUFSIZE).decode('cp1252')
        if (time.time() - start) > 1:
            LOG.debug("Timeout waiting for data")
            break

    if result.endswith(LAST_DELIMITER[0]):
        LOG.debug("RADIO->PC: %s" % result.strip())
        result = result[:-1]        # remove delimiter
    else:
        LOG.debug("Giving up")

    return result.strip()

def command(port: serial.serialposix.Serial, cmd: str, *args) -> str:
    """ send serial command inside a LOCK-protected spot """
    with LOCK:
        return _command(port, cmd, *args)

def get_id(port: serial.serialposix.Serial) -> str:
    """ Get the ID and type of the radio attached to @ser
        port    is the serial port to use
        returns the model string of the radio
    """
    global LAST_BAUD
    bauds = [4800, 9600, 19200, 38400, 57600, 115200]
    bauds.remove(LAST_BAUD)
    # Make sure LAST_BAUD is last so that it is tried first below
    bauds.append(LAST_BAUD)

    global LAST_DELIMITER
    command_delimiters = [(";", "")]

    for delimiter in command_delimiters:
        # Process the baud options in reverse order so that we try the
        # last one first, and then start with the high-speed ones next
        for i in reversed(bauds):
            LAST_DELIMITER = delimiter
            LOG.info(
            "Trying ID at baud %d with delimiter '%s'", i, delimiter)
            port.baudrate = i
            port.write(LAST_DELIMITER[0].encode())
            port.read(25)
            try:
                resp = command(port, IDEN_CMD)
            except UnicodeDecodeError:
                # If we got binary here, we are using the wrong rate
                # or not talking to a elecraft live radio.
                continue

            # most elecraft radios
            if " " in resp:
                LAST_BAUD = i
                return resp.split(" ")[1]

            # Radio responded in the right baud rate,
            # but threw an error because of all the crap
            # we have been hurling at it. Retry the ID at this
            # baud rate, which will almost definitely work.
            if "?" in resp:
                resp = command(port, IDEN_CMD)
                LAST_BAUD = i
                if " " in resp:
                    return resp.split(" ")[1]

            # elecraft radios that return ID numbers, ask for more info
            if resp in RADIO_IDS:
                xt = command(port, XTRA_CMD)
                if "4---" == xt[-4:]:
                    resp = "ID0174"
                elif "--" == xt[-2:]:
                    resp = "ID017X"
                elif "01" == xt[-2:]:
                    resp = "ID0171"
                elif "02" == xt[-2:]:
                    resp = "ID0172"
                return RADIO_IDS[resp]

    raise errors.RadioError("No response from radio")

def do_download(port : serial.serialposix.Serial) -> (list, list, list):
    """ This queries radio port for all regular and special memories
        then converts ASCII to Elecraft internal format
        textmap is the ascii return from commanded reads
        Changed is boolean map of textmap elements as they get changed
                It starts as False, indicating it is as read from radio
        bmap    is the binary bitmap created from the textmaps.
                It doesn't have address and length prepended
        pmap    is the list of parsed data corresponding to bmap
                It includes address, length and checksum
        bmap, pmap and Changed are the returned values
    """
    # Get the serial port connection
    # textmap = list()
    Changed = list()
    bmap = bytearray()
    pmap = list()
    # For each regular memory, read radio and convert to memory image
    for _i in range(0, 100):
        _t = command(port, _cmd_get_memory(_i))
        # textmap.append(_t)
        _b = bytearray.fromhex(_t[8:])
        bmap += _b
        _c = ElecraftMem(*struct.unpack(FMT, bytearray.fromhex(_t[2:])))
        pmap.append(_c)
        Changed.append(False)
    return bmap, pmap, Changed

def _cmd_get_memory(number: int|str) -> str:
    """ returns a string with radio command to read a memory from radio.
        The return includes a one-byte checksum
        """
    address = MEM_START + number * MEM_LEN
    message = f"{address:04x}{MEM_LEN:02x}"
    checksum = ((sum(bytearray.fromhex(message)) - 1) & 0xFF) ^ 0xFF
    message = f"{READ_CMD}{message}{checksum:2x}"
    return message

def _cmd_set_memory(number: int, mem: str = None) -> str:
    """ returns a string with the appropriate write command 
        number  is the CHIRP memory number
        mem     is the string to include in the command
        """
    address = MEM_START + number * MEM_LEN
    length = MEM_LEN
    message = f"{address:04x}{length:02x}{mem:0x}"
    checksum = ((sum(bytearray.fromhex(message)) - 1) & 0xFF) ^ 0xFF
    return f"{WRIT_CMD}{message}{checksum:02x}"

def radio_to_freq(b: bytes) -> int:
    """ Turns the five hex bytes into integer frequency """
    if b[0] == b'\xFF':
        return -1
    freq = b[0]*1000000 + b[1]*10000 + b[2]*100 + b[3]*10 + b[4]
    return freq

def freq_to_radio(f: int) -> bytes:
    """ Turn an integer frequency into an Elecraft 5-byte string """
    byt = b'\x00\x00\x00\x00\x00'
    if f == -1:
        return b'\xff\xff\xff\xff\xff'
    _f = f
    bm = (_f//1000000).to_bytes(1, "big")
    _f -= int.from_bytes(bm) * 1000000
    bT = (_f // 10000).to_bytes(1, "big")
    _f -= int.from_bytes(bT) * 10000
    bt = (_f //100).to_bytes(1, "big")
    _f -= int.from_bytes(bt) * 100
    bh = (_f // 10).to_bytes(1, "big")
    _f -= int.from_bytes(bh) * 10
    bo = _f.to_bytes(1, "big")
    byt = bm+ bT +bt + bh + bo
    return byt


@directory.register
class ElecraftRadio(chirp_common.CloneModeRadio):
    """ Base class for all KX-series Elecraft Radios """
    VENDOR = "Elecraft"
    MODEL = "K[X]-series"
    BAUD_RATE = 38400
    NEEDS_COMPAT_SERIAL = False

    def __init__(self, *args, **kwargs) -> None:
        chirp_common.CloneModeRadio.__init__(self, *args, **kwargs)
        if self.pipe:
            self.pipe.timeout = 0.1
            # Turn off Auto-info Mode, so no radio interruptions
            command(self.pipe, AUTO_CMD,  "0")
            # Ask the radio to identify itself
            radio_id = get_id(self.pipe)
            if radio_id != self.MODEL.split(" ", 1)[0]:
                raise errors.RadioError(
                    f"Radio reports {radio_id} not {self.MODEL}")
        self.get_features()
        print("__init__", repr(self))

    # Return information about this radio's features, including
    # how many memories it has, what bands it supports, etc
    def get_features(self) -> chirp_common.RadioFeatures:
        rf = chirp_common.RadioFeatures()
        rf.has_bank = False
        rf.has_bank_index = False
        rf.has_bank_names = False
        rf.has_comment = True
        rf.has_ctone = False
        rf.has_dtcs = False
        rf.has_dtcs_polarity = False
        rf.has_mode = True
        rf.has_name = True
        rf.has_nostep_tuning = True
        rf.has_offset = True
        rf.has_settings = False
        rf.has_tuning_step = False
        rf.memory_bounds = (0, 99)  # This radio supports memories 0-99
        rf.valid_bands = [(310000, 32000000),   # Valid for KX3 w/o xvrtr
                      (44000000, 54000000),
                      ]
        rf.valid_modes = ["AM", "USB", "LSB", "CW", "FM", "FSK", "RTTY",
                   "RTTYR", "FSKR"]
        rf.valid_name_length = 5
        return rf

    def do_upload(self) -> None:
        """This is your upload function"""
        port = self.pipe
        for i in range(0, 100):
            if self.Changed[i]:
                mem = chirp_common.Memory[i]
                # TODO pack the memory back and hex it
                byt = struct.pack(FMT, mem)
                status = command(port, _cmd_set_memory(mem.number), byt)
                self.Changed[i] = False

    # Do a download of the radio from the serial port
    def sync_in(self):
        # DAR print("Sync_in _mmap. This will be slow!", type(self.pipe))
        (self._mmap, self._pmap, self.Changed) = do_download(self.pipe)
        # DAR print("sync_in:", repr(self._mmap))
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

    # Do an upload of the radio to the serial port
    def sync_out(self):
        self.do_upload()

    # Return a raw representation of the memory object, which
    # is very helpful for development
    def get_raw_memory(self, number):
        return repr(self._memobj.memory[number])

    # Extract a CHIRP-level memory object from the radio-level memory map
    # This is called to populate a memory in the UI
    def get_memory(self, number: int|str) -> chirp_common.Memory:
        # Get a radio's memory object mapped to the image.
        # This is a NamedTuple
        _mem = self._pmap[number]
        # DAR print("\nget_memory: ", number, _mem)
        # DAR print("\rvfoa:", _mem.rvfoa)
        _rvfoa = _mem.rvfoa
        # Create a CHIRP-level memory object to return to the UI
        mem = chirp_common.Memory()

        mem.number = number                 # Set the CHIRP memory number
        if (_mem.rvfoa == b'\xff\xff\xff\xff\xff' and
             _mem.rvfoa == b'\xff\xff\xff\xff\xff') or \
           ( _mem.rvfoa == b'\xff\xff\xff\xff\xff' and
             _mem.rvfoa == b'\xff\xff\xff\xff\xff'):
            mem.empty = True
            mem.freq = 0
            mem.offset = 0
            mem.mode = MODES[0]
            return mem
        # Convert your low-level frequency to Hertz
        mem.freq = radio_to_freq(_mem.rvfoa)
        mem.offset = radio_to_freq(_mem.rvfob)
        # for VFOA
        mem.mode = MODES[_mem.modes & 0x0f]
        # DAR print(_mem.modes, bytes(_mem.modes))
        mem.name = _mem.rlabel.decode('cp1252').translate(XTABLE)
        mem.rtone = chirp_common.TONES[_mem.subtone]
        # DAR print(number, repr(mem))
        return mem

    # Store details about a CHIRP-level memory to the radio's map
    # This is called when a user edits a memory in the UI
    def set_memory(self, memory: chirp_common.Memory) -> None:
        # Get a low-level memory object mapped to the image
        _mem = self._pmap[memory.number]

        # Convert to low-level frequency representation
        _mem.rvfoa = freq_to_radio(memory.freq)
        _mem.rvfob = freq_to_radio(memory.offset)
        _mem.rlabel = memory.name.translate(REVTABLE)
        _mem.modes = index(memory.mode, MODES)
        _mem.subtone = memory.tone
        self.Changed[memory.number] = True


@directory.register
class KX3Radio(ElecraftRadio):
    """ Class for Elecraft KX3 radio. DAR"""
    MODEL = "KX3"


@directory.register
class KX2Radio(ElecraftRadio):
    """ Class for Elecraft KX2 radio.  DAR"""
    MODEL = "KX2"

    def get_features(self):
        super().get_features()
        rf = chirp_common.RadioFeatures()
        rf.valid_modes = ["AM", "USB", "LSB", "CW", "FM", "FSK", "RTTY"]
        rf.valid_bands = [(500000, 32000000),   # Valid for KX2 w/o xvrtr
                          ]
        return rf


@directory.register
class KXRadio(ElecraftRadio):
    """ Class for Elecraft KX radio. DAR"""
    MODEL = "K3"


@directory.register
class K4Radio(ElecraftRadio, chirp_common.ExperimentalRadio):
    """ Class for Elecraft K4 radio. DAR"""
    MODEL = "K4"
