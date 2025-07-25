""" A toy but functioning driver to build from """
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

from chirp import chirp_common, directory, memmap
from chirp import bitwise
from chirp.settings import RadioSetting, RadioSettingGroup,\
                           RadioSettingValueList, RadioSettingValueString

# Here is where we define the memory map for the radio. Since
# We often just know small bits of it, we can use #seekto to skip
# around as needed.
#
# Our fake radio includes just a single array of ten memory objects,
# With some very basic settings, a 32-bit unsigned integer for the
# frequency (in Hertz) and an eight-character alpha tag
# Also supports TemplateExtra with mem.extra fields
#
MEM_FORMAT = """
#seekto 0x0000;
struct {
  u32 freq;
  u32 vfoa;
  u32 vfob;
  u8  modeb:4,
      modea:4;
  char name[8];
} memory[10];
"""


def do_download(radio):
    """This is your download function"""
    # NOTE: Remove this in your real implementation!
    return memmap.MemoryMapBytes(b"\x00" * 1000)

    # Get the serial port connection
    # serial = radio.pipe

    # Our fake radio is just a simple download of 1000 bytes
    # from the serial port. Do that one byte at a time and
    # store them in the memory map
    # data = b""
    # for _i in range(0, 1000):
    #     data += serial.read(1)

    # return memmap.MemoryMapBytes(data)


def do_upload(radio):
    """This is your upload function"""
    # NOTE: Uncomment this in your real implementation!
    # raise Exception("This template driver does not really work!")

    # Get the serial port connection
    serial = radio.pipe

    # Our fake radio is just a simple upload of 1000 bytes
    # to the serial port. Do that one byte at a time, reading
    # from our memory map
    for i in range(0, 1000):
        serial.write(radio.get_mmap()[i])


# Uncomment this to actually register this radio in CHIRP
@directory.register
class TemplateRadio(chirp_common.CloneModeRadio):
    """Acme Template"""
    VENDOR = "Acme"     # Replace this with your vendor
    MODEL = "Template"  # Replace this with your model
    BAUD_RATE = 9600    # Replace this with your baud rate

    # All new drivers should be "Byte Clean" so leave this in place.

    # Return information about this radio's features, including
    # how many memories it has, what bands it supports, etc
    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_bank = False
        rf.memory_bounds = (0, 9)  # This radio supports memories 0-9
        rf.valid_bands = [(144000000, 148000000),  # Supports 2-meters
                          (440000000, 450000000),  # Supports 70-centimeters
                          ]
        return rf

    # Do a download of the radio from the serial port
    def sync_in(self):
        self._mmap = do_download(self)
        self.process_mmap()

    # Do an upload of the radio to the serial port
    def sync_out(self):
        do_upload(self)

    # Convert the raw byte array into a memory object structure
    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

    # Return a raw representation of the memory object, which
    # is very helpful for development
    def get_raw_memory(self, number: int) -> str:
        return repr(self._memobj.memory[number])

    # Extract a high-level memory object from the low-level memory map
    # This is called to populate a memory in the UI
    def get_memory(self, number: int|str) -> chirp_common.Memory:
        # Get a low-level memory object mapped to the image
        _mem = self._memobj.memory[number]

        # Create a high-level memory object to return to the UI
        mem = chirp_common.Memory()

        mem.number = number                 # Set the memory number
        # Convert your low-level frequency to Hertz
        mem.freq = int(_mem.freq)
        _s = str(_mem.name)
        if not _s.isprintable():
            mem.name = ' '
        else:
            mem.name = _s.rstrip()  # Set the alpha tag

        # We'll consider any blank (i.e. 0 MHz frequency) to be empty
        if mem.freq == 0:
            mem.empty = True

        return mem

    # Store details about a high-level memory to the memory map
    # This is called when a user edits a memory in the UI
    def set_memory(self, mem: chirp_common.Memory) -> None:
        # Get a low-level memory object mapped to the image
        _mem = self._memobj.memory[mem.number]

        # Convert to low-level frequency representation
        _mem.freq = mem.freq
        _mem.name = mem.name.ljust(8)[:8]  # Store the alpha tag


@directory.register
class TemplateRadioExtra(TemplateRadio):
    """ Acme template but with mem.extra fields """
    MODEL = "TemplateExtra"  # Replace this with your model
    MODES =  ['CW', 'AM', 'FM']

    def get_features(self) -> chirp_common.RadioFeatures:
        rf = chirp_common.RadioFeatures()
        rf.has_bank = False
        rf.has_ctone = False
        rf.has_dtcs = False
        rf.has_dtcs_polarity = False
        rf.has_mode = False
        rf.has_nostep_tuning = True
        rf.has_offset = False
        rf.has_rx_dtcs = False
        rf.has_tuning_step = False
        rf.has_variable_power = True
        rf.memory_bounds = (0, 9)  # This radio supports memories 0-9
        rf.requires_call_lists = False
        rf.valid_bands = [(0,1),
                          (144000000, 148000000),  # Supports 2-meters
                          (440000000, 450000000),  # Supports 70-centimeters
                          ]
        rf.valid_modes = self.MODES
        rf.valid_name_length = 8
        rf.valid_power_levels = [chirp_common.PowerLevel('min', watts = 0.0),\
                                 chirp_common.PowerLevel('max', watts = 15.0)]
        return rf

    def get_memory(self, number: int|str) -> chirp_common.Memory:
        # Get a Radio memory object mapped to the image
        rmem = self._memobj.memory[number]

        # Create a CHIRP memory object to return to the UI
        mem = chirp_common.Memory()

        mem.number = number                 # Set the memory number
        # Convert your Radio frequency to Hertz
        mem.freq = int(rmem.freq)

        _s = str(rmem.name)
        mem.name = ' '
        if _s.isprintable():
            mem.name = _s.rstrip()  # Set the alpha tag

        # We'll consider any blank (i.e. 0 MHz frequency) to be empty
        if mem.freq == 0:
            mem.empty = True

        vfoa = rmem.vfoa
        modea = rmem.modea
        mem.extra = RadioSettingGroup('extra', 'Extra')
        _rs = RadioSetting('vfoa', 'VFO A (MHz)',
              RadioSettingValueString(0, 10, chirp_common.format_freq(vfoa)))
        mem.extra.append(_rs)
        _rs = RadioSetting('modea', 'Mode A',
              RadioSettingValueList(self.MODES, None, modea))
        mem.extra.append(_rs)

        vfob = rmem.vfob
        modeb = rmem.modeb
        _rs = RadioSetting('vfob', 'VFO B (MHz)',
              RadioSettingValueString(0, 10, chirp_common.format_freq(vfob)))
        mem.extra.append(_rs)
        _rs = RadioSetting('modeb', 'Mode B',
              RadioSettingValueList(self.MODES, None, modeb))
        mem.extra.append(_rs)

        return mem

    # Store details about a CHIRP memory to the Radio memory map
    # This is called when a user edits a memory in the UI
    def set_memory(self, mem: chirp_common.Memory) -> None:
        # Get the corresponding Radio memory object
        rmem = self._memobj.memory[mem.number]

        # Convert CHIRP to Radio level frequency representation
        rmem.freq = mem.freq
        rmem.name = mem.name.ljust(8)[:8]  # Store the alpha tag
        rmem.vfoa = chirp_common.parse_freq(str(mem.extra['vfoa'].value))
        rmem.vfob = chirp_common.parse_freq(str(mem.extra['vfob'].value))
        rmem.modea = mem.extra['modea'].value
        rmem.modeb = mem.extra['modeb'].value
