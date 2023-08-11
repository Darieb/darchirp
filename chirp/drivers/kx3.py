""" CHIRP driver for Elecraft KX-series and K3 """
# Copyright 2010 Dan Smith <dsmith@danplanet.com> (kenwood_live.py)
# Copyright 2023 Declan Rieb <WD5EQY@arrl.net>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import threading
import os
import time
import logging
import string

from typing import NamedTuple
from struct import pack, unpack, calcsize
from chirp import chirp_common, errors, directory
from chirp.settings import RadioSetting, RadioSettingGroup,\
                           RadioSettingValueList, RadioSettingValueBoolean

logging.basicConfig(level="debug")
LOG = logging.getLogger(__name__)

NOCACHE = "CHIRP_NOCACHE" in os.environ


class ElecraftMem(NamedTuple):
    """ Elecraft memory format (unpacked from 0x40 bytes) """
    address: int = 0x0c00
    length: int = 0x40
    rvfoa: bytes = b'\xff\xff\xff\xff\xff'
    rvfob: bytes = b'\xff\xff\xff\xff\xff'
    modes: int = 0xff
    dmode: int = 0xff
    f2: int = 0xff
    f3: int = 0xff
    f4: int = 0xff
    band: int = 0xff
    subtone: int = 0xff
    offset: int = 0xff
    repflag: int = 0xff
    f9: int = 0xff
    fa: int = 0xff
    fb: int = 0xff
    fc: int = 0xff
    fd: int = 0xff
    rlabel: bytes = b'\xff\xff\xff\xff\xff'
    comment: bytes = 24 * b'\xff'
    cksum: int = -1


"""
Magic for decoding the radio's responses are in FMT here,
    and match the ElecraftMem structure above.:
    Assumes 68 characters, converted from text hex string of 136 chars
    >   Big-endian
    H   unsigned short (2 bytes):  address
    B   Unsigned char-sized integer: length
    5s  string, freq spec (two one per VFO)
    B   unsigned char-sized integer: Modes
    13B for other flags
    x   5 ignored
    5s  string, label (in vc format)
    24s string, comment field (in ascii)
    i   4-byte signed integer for checksum
"""
FMT = '>HB5s5sB13B8x5s24si'

DUPLEX = {0: "", 1: "+", 2: "-", 3: "split"}
MODES = {0: "CW", 1: "LSB", 2: "USB", 3: "DN", 4: "AM", 5: "FM", 6: "Auto"}
DUMMY_MODE = "Auto"
SUB_MODES = ['DATA-A', 'AFSK-A', 'FSK-D', 'PSK-D', 'Auto']
ANTENNAS = ['Ant1', 'Ant2']

ELECRAFT_TONES = list(chirp_common.TONES) + ["1750.0"]
# All Elecraft commands driver uses:
READ_CMD = "ER"     # MC on K4
WRIT_CMD = "EW"     # MC on K4
IDEN_CMD = "ID"
XTRA_CMD = "OM"
AUTO_CMD = "AI"
EMPTY_DATA = 126 * b'\xFF'

BAND_NAMES = [
    "160",
    "80",
    "60",
    "40",
    "30",
    "20",
    "17",
    "15",
    "12",
    "10",
    "6",
    # reserved "bands"
    "Rs1",
    "Rs2",
    "Rs3",
    "Rs4",
    "Rs5",
    # transverter "bands"
    "XV1",
    "XV2",
    "XV3",
    "XV4",
    "XV5",
    "XV6",
    "XV7",
    "XV8",
    "XV9"
    ]
SPECIALS = [f"{BAND_NAMES[(n // 4)]} M{(n % 4) + 1}" for n in range(0, 100)]
MEM_START = 0x0C00
MEM_LEN = 0X40
VALID_CHARS = ' ' + string.ascii_uppercase + string.digits + '*+/@_'
XTABLE = dict(enumerate(VALID_CHARS))
REVTABLE = {v: k for k, v in XTABLE.items()}
# for index in range(0, len(VALID_CHARS)):
#     XTABLE[index] = VALID_CHARS[index]
#     REVTABLE[VALID_CHARS[index]] = index

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


def _command(ser, cmd, *args):
    # Send @cmd to radio via @ser
    global LOCK, LAST_DELIMITER, COMMAND_RESP_BUFSIZE

    start = time.time()

    # yet to do: This global use of LAST_DELIMITER breaks reentrancy
    # and needs to be fixed.
    if args:
        cmd += LAST_DELIMITER[1] + LAST_DELIMITER[1].join(args)
    cmd += LAST_DELIMITER[0]

    LOG.debug("PC->RADIO: %s", cmd.strip())
    ser.write(cmd.encode('cp1252'))

    result = ""
    while not result.endswith(LAST_DELIMITER[0]):
        result += ser.read(COMMAND_RESP_BUFSIZE).decode('cp1252')
        if (time.time() - start) > 1:
            LOG.debug("Timeout waiting for data")
            break

    if result.endswith(LAST_DELIMITER[0]):
        LOG.debug("RADIO->PC: %s", result.strip())
        result = result[:-1]
    else:
        LOG.debug("Giving up")

    return result.strip()


def command(ser, cmd, *args):
    """ send serial command inside a LOCK-protected spot """
    with LOCK:
        return _command(ser, cmd, *args)


def get_id(ser):
    """ Get the ID of the radio attached to @ser """
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
            ser.baudrate = i
            ser.write(LAST_DELIMITER[0].encode())
            ser.read(25)
            try:
                resp = command(ser, IDEN_CMD)
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
                resp = command(ser, IDEN_CMD)
                LAST_BAUD = i
                if " " in resp:
                    return resp.split(" ")[1]

            # elecraft radios that return ID numbers
            if resp in RADIO_IDS:
                xt = command(ser, XTRA_CMD)
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


def get_tmode(tone, ctcss, dcs):
    """ Get the tone mode based on the values of the tone, ctcss, dcs """
    if dcs and int(dcs) == 1:
        return "DTCS"
    if int(ctcss):
        return "TSQL"
    if int(tone):
        return "Tone"
    return ""


def iserr(result):
    """ Returns True if the @result from a radio is an error """
    return result in ["N", "?"]


class ElecraftLiveRadio(chirp_common.LiveRadio):
    """ Base class for all live-mode Elecraft radios """
    BAUD_RATE = 38400
    VENDOR = "Elecraft"
    MODEL = "K-series"
    NEEDS_COMPAT_SERIAL = False
    # HARDWARE_FLOW = True      # needed for elecraft

    _vfo = 0
    _upper = 99
    _has_name = True
    _modes = list(MODES.values())
    _bands = [(310000, 32000000),       # valid for KX3 w/o xverter
              (44000000, 54000000)]

    def __init__(self, *args, **kwargs):
        chirp_common.LiveRadio.__init__(self, *args, **kwargs)
        self._memcache = {}
        self._rmemcache = {}
        if self.pipe:
            self.pipe.timeout = 0.1
            radio_id = get_id(self.pipe)
            if radio_id != self.MODEL.split(" ", 1)[0]:
                raise errors.RadioError(
                    f"Radio reports {radio_id} not {self.MODEL}")
            # Turn off Auto-info Mode, so no radio interruptions
            command(self.pipe, AUTO_CMD,  "0")

    def get_features(self) -> chirp_common.RadioFeatures:
        """ Tell CHIRP what features this radio has """
        rf = chirp_common.RadioFeatures()
        rf.can_odd_split = True
        rf.has_bank = False
        # rf.has_ctone = True
        rf.has_ctone = False
        rf.has_dtcs = False
        rf.has_dtcs_polarity = False
        rf.has_name = True
        rf.has_settings = False
        rf.has_offset = True
        rf.has_mode = True
        # rf.has_tuning_step = True
        rf.has_tuning_step = False
        rf.has_nostep_tuning = True
        # rf.has_cross = True
        rf.has_cross = False
        rf.has_comment = True
        rf.memory_bounds = (0, self._upper)
        rf.valid_bands = self._bands
        rf.valid_characters = chirp_common.CHARSET_UPPER_NUMERIC + " *+-/"
        rf.valid_duplexes = ["", "-", "+", "split"]
        if rf.has_cross:
            rf.valid_cross_modes = ["Tone->Tone", "->Tone"]
        rf.valid_modes = list(self._modes)
        rf.valid_name_length = 5
        rf.valid_skips = []
        rf.valid_special_chans = SPECIALS
        rf.valid_tones = ELECRAFT_TONES
        rf.valid_tmodes = ["", "Tone", "TSQL", "Cross"]
        return rf

    def _cmd_get_memory(self, number: int) -> str:
        """ returns a string with command to read a memory from radio """
        address = MEM_START + number * MEM_LEN
        message = f"{address:04x}{MEM_LEN:02x}"
        checksum = ((sum(bytearray.fromhex(message)) - 1) & 0xFF) ^ 0xFF
        message = f"{READ_CMD}{message}{checksum:2x}"
        return message

    def _cmd_set_memory(self, number: int, byt: bytes) -> str:
        """ returns string with command to write memory number to radio """
        address = MEM_START + number * MEM_LEN
        checksum = int((sum(byt[:-1]) - 1) & 0xFF) ^ 0xFF
        checksum += (address & 0xff) + ((address >> 8) & 0xff) + MEM_LEN
        checksum = (checksum & 0xFF) ^ 0xFF
        message = byt[:-1].hex()
        cmd = f"{WRIT_CMD}{address:04x}{MEM_LEN:02x}{message}{checksum:02x}"
        LOG.warning("CSM: %s %d bytes. %s", message, len(message), cmd)
        return cmd

    def radio_to_freq(self, byt: str) -> int:
        """ Turns the hex "byt" into integer frequency """
        if byt[0] == 0xFF:
            return -1
        freq = byt[0]*1000000 +\
            byt[1]*10000 +\
            byt[2]*100 +\
            byt[3]*10 +\
            byt[4]
        return freq

    def freq_to_radio(self, freq: int) -> bytes:
        """ Turns the frequency "freq" to Elecraft format """
        ss = chr(freq // 1000000)
        rem = freq % 1000000
        ss += chr(rem // 10000)
        rem = rem % 1000
        ss += chr(rem // 100)
        rem = rem % 100
        ss += chr(rem // 10)
        ss += chr(rem % 10)
        return bytes(ss, 'cp1252')

    def ret_empty(self, number: int, mem: chirp_common.Memory) ->\
            chirp_common.Memory:
        """ build an empty memory for return """
        mem.number = number
        mem.empty = True
        self._memcache[number] = mem
        if number > self._upper:
            mem.extd_number = SPECIALS[number - self._upper - 1]
        return mem

    def get_memory(self, number: int | str) -> chirp_common.Memory:
        """ Returns Memory instance corresponding with "number" """
        if number in self._memcache and not NOCACHE:
            return self._memcache[number]
        mem = chirp_common.Memory()
        mem.number = number
        if isinstance(number, str):
            mem.number = SPECIALS.index(number) + self._upper + 1
            mem.extd_number = number
        # after this point, mem.number contains the canonical memory index
        result = command(self.pipe, *self._cmd_get_memory(mem.number))
        ln = len(result)
        if result in "NE?":
            LOG.warning("Radio returned error '%s'", result)
            return self.ret_empty(mem.number, mem)
        if READ_CMD not in result:
            LOG.error("Not sure what to do with this: %s", result)
            raise errors.RadioError("Unexpected result returned from radio")
        byt = bytearray.fromhex(result[2:])
        ckadd = ((int(sum(byt)) - 1) ^ 0xFF) & 0xFF
        if ckadd != 0:
            LOG.warning("Checksum bad (get_memory). %x should be zero.", ckadd)
            return self.ret_empty(mem.number, mem)
        if ln <= 11:
            # Response is short (no data), ignore
            LOG.debug("Radio returned no valid data '%s'", result)
            return self.ret_empty(mem.number, mem)
        if ln == 138:
            # _mem is the elecraft memory mapped image, mem is the CHIRP memory
            _mem = ElecraftMem(*unpack(FMT, byt[:calcsize(FMT)]))
            vfoa = self.radio_to_freq(_mem.rvfoa)
            if vfoa < 0:
                mem.empty = True
                return mem
            vfob = self.radio_to_freq(_mem.rvfob)
            modea = MODES.get(_mem.modes & 0x0F, DUMMY_MODE)
            modeb = MODES.get(((_mem.modes & 0xF0) >> 4) & 0x0F, DUMMY_MODE)
            dmode = _mem.dmode ^ 0x0F
            if dmode < len(SUB_MODES) and (modea == 'DN' or modeb == 'DN'):
                dmode = SUB_MODES[dmode]
            else:
                dmode = DUMMY_MODE
            mem.freq = vfoa
            mem.mode = modea
            mem.offset = vfob
            mem.duplex = "split"
            mem.name = _mem.rlabel.decode('cp1252').translate(XTABLE)
            if 0 < _mem.subtone <= len(ELECRAFT_TONES):
                mem.rtone = mem.ctone = ELECRAFT_TONES[_mem.subtone]
                # Fake "Tone" mode to get CHIRP to show VFOB as OFFSET
                mem.tmode = "Tone"
            mem.comment = _mem.comment.decode('cp1252')\
                .strip('\x00').strip('\xFF')
            self.get_mem_extra(_mem, mem, modeb)
        self._rmemcache[mem.number] = _mem
        self._memcache[mem.number] = mem
        LOG.debug("GLM: %s", mem)
        return mem

    def get_mem_extra(self, _mem: ElecraftMem, mem: chirp_common.Memory,
                      modeb: str) -> None:
        """ Save/display extra fields according to Tuple y into memory mem """
        mem.extra = RadioSettingGroup('Extra', 'extra')
        val = RadioSettingValueList(self._modes, modeb)
        rs = RadioSetting('modeb', 'ModeB', val)
        mem.extra.append(rs)
        val = RadioSettingValueList(SUB_MODES,
                                    SUB_MODES[(_mem.dmode >> 4) & 0x0F])
        rs = RadioSetting('dmode', 'D Mode', val)
        mem.extra.append(rs)
        reverse = (_mem.f2 & 0x40) != 0
        val = RadioSettingValueBoolean(reverse)
        rs = RadioSetting('rev', 'Reverse', val)
        mem.extra.append(rs)
        antenna = ANTENNAS[(_mem.f2 & 0x80) >> 7]
        val = RadioSettingValueList(ANTENNAS, antenna)
        rs = RadioSetting('antenna', 'Antenna', val)
        mem.extra.append(rs)

    def set_memory(self, memory: chirp_common.Memory) -> None:
        """ Save a radio memory that's stored in CHIRP "memory" """
        if memory.empty:
            byt = EMPTY_DATA
        else:
            _mem = self._rmemcache.get(memory.number, ElecraftMem())
            modeb = int(memory.extra['modeb'].value) if 'modeb' in memory.extra\
                else 0
            mydmode = str(memory.extra['dmode'].value)
            LOG.warning("SM: mydmode(%s)=%s SUBMODES=%s",
                type(mydmode), mydmode, SUB_MODES)
            mydmode = SUB_MODES.index(mydmode)
            mydmode = mydmode if mydmode < 6 else 0
            mydmode = ((mydmode << 4) & 0xF0) | (int(_mem.dmode) & 0x0F)
            myf2a = 0x80 if str(memory.extra['antenna'].value) == 'Ant2' else 0
            myf2r = 0x40 if str(memory.extra['rev'].value) == 'Rev' else 0
            myf2 = (_mem.f2 & 0x37) | myf2a | myf2r
            _mem._replace(
                    rvfoa=self.freq_to_radio(memory.freq),
                    rvfob=self.freq_to_radio(memory.offset),
                    modes=(((modeb & 0x0F) << 4) | (int(memory.mode) & 0x0F)),
                    dmode=mydmode,
                    f2=myf2,
                    rlabel=memory.name.translate(REVTABLE),
                    comment=memory.comment,
                    subtone=ELECRAFT_TONES.index(str(memory.rtone))
                    )
            byt = pack(FMT, *_mem)
        cmd = self._cmd_set_memory(memory.number, byt)
        LOG.warning("SM: byt='%s'\nlen=%d, command='%s'",
                    byt, len(cmd), cmd)
        # r1 = command(self.pipe, *self._cmd_set_memory(mem.number), byt)
        r1 = 'EW goes here'     # But not yet!
        if r1 in "NE?":
            LOG.warning("set_memory: Radio returned error '%s'", r1)

    def erase_memory(self, number: int) -> None:
        """ Erase the memory at "number" """
        if number not in self._memcache:
            return
        # send an erase command here ... yet to do
        resp = command(self.pipe, *self._cmd_set_memory(number, EMPTY_DATA))
        if iserr(resp):
            raise errors.RadioError(f"Radio refused delete of {number}")
        del self._memcache[number]


@directory.register
class KX3Radio(ElecraftLiveRadio):
    """ Class for Elecraft KX3 radio. DAR"""
    MODEL = "KX3"


@directory.register
class KX2Radio(ElecraftLiveRadio):
    """ Class for Elecraft KX2 radio.  DAR"""
    MODEL = "KX2"
    # No FM in this radio
    MODES = {0: "CW", 1: "LSB", 2: "USB", 3: "DN", 4: "AM"}


@directory.register
class KXRadio(ElecraftLiveRadio):
    """ Class for Elecraft KX radio. DAR"""
    MODEL = "K3"


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
struct elecraft_freq vfoa;
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
// If this were real memory: #skipto 0x0C00;
struct {
    struct memslot slot;
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
    i32 cheksum;        // Calculation done on last byte, but sign extended
} radiomem[99];

// If this were real memory: #skipto 0x0100;
struct memslot normal_band[11];
// If this were real memory: #skipto 0x0200;
struct memslot xcvr[9]; // frequencies are IF; read state for more

// If this were real memory: #skipto 0x02A2;
struct {
    u16 state[5];
} transverter_state[9]; // unknown, but probably contain mixer frequency
"""
