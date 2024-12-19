""" Driver for Yaesu FT-1D
    Basis for Yaesu FT2DR and FT-3DR and FTM-3200
"""
# Copyright 2010 Dan Smith <dsmith@danplanet.com>
# Copyright 2014 Angus Ainslie <angus@akkea.ca>
# Copyright 2023, 2024 Declan Rieb <WD5EQY@arrl.net>
# Sections of digital settings applied from ft70.py, thus
# Copyright 2017 Nicolas Pike <nick@zbm2.com>
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

import re
import string
import logging
import traceback

from chirp.drivers import yaesu_clone
from chirp import chirp_common, directory, bitwise
from chirp import memmap
from chirp.settings import RadioSettingGroup, RadioSetting, RadioSettings, \
            RadioSettingValueInteger, RadioSettingValueString, \
            RadioSettingValueList, RadioSettingValueBoolean, \
            InvalidValueError
from chirp import util
try:
    from chirp.chirp_common import SpecialBankModelInterface
    SPECIALS_IN_BANKS = True
except ImportError:
    SPECIALS_IN_BANKS = False
LOG = logging.getLogger(__name__)

MEM_SETTINGS_FORMAT = """
#seekto 0x047e;
struct {
  u8 unknown1;
  u8 flag;
  u16 unknown2;
  struct {
    u8 padded_yaesu[16];
  } message;
} opening_message;

#seekto 0x049a;
struct {
  u8 vfo_a;
  u8 vfo_b;
} squelch;

#seekto 0x04c1;
struct {
  u8 beep;
} beep_select;

#seekto 0x04ce;
struct {
  u8 lcd_dimmer;        // 14 DIMMER
  u8 dtmf_delay;        // 18 DT DLY
  u8 unknown0[3];
  u8 unknown1:4,
     lcd_contrast:4;
  u8 lamp;              // 28 LAMP
  u8 lock;              // 30 LOCK
  u8 unknown2;
  u8 mic_gain;          // 31 MCGAIN
  u8 unknown2_3;
  u8 dw_interval;       // 21 DW INT
  u8 ptt_delay;         // 42 PTT.DLY
  u8 rx_save;           // 48 RX.SAVE
  u8 scan_restart;      // 53 SCN.STR
  u8 unknown3;
  u8 scan_resume;
  u8 unknown4[5];
  u8 tot;
  u8 unknown5[3];
  u8 vfo_mode:1,        // 60 VFO.MOD
     unknown6:1,
     scan_lamp:1,       // 51 SCN.LMP
     unknown7:1,
     ars:1,             // 45 RPT.ARS
     dtmf_speed:1,      // 20 DT SPD
     unknown8:1,
     dtmf_mode:1;
  u8 busy_led:1,
     unknown9:2,
     bclo:1,            // 03 BCLO
     beep_edge:1,       // 06 BEP.EDG
     unknown9_1:3;
  u8 unknown10:5,
     password:1,
     home_rev:1,        // 26 HOME/REV
     moni:1;            // 32 MON/T-Call
  u8 gm_interval:4,     // 25 GM INT
     unknown11:4;
  u8 vol_mode:1,
     unknown11_1:7;
  u8 unknown12:4,
     home_vfo:1,        // 27 HOME->VFO
     unknown12_1:2,
     dw_rt:1;           // 23 DW RVT;
} scan_settings;

#seekto 0x54a;
struct {
    u16 in_use;
} bank_used[24];

#seekto 0x064a;
struct {
  u8 unknown0[4];
  u8 frequency_band;
  u8 unknown1:6,
     manual_or_mr:2;
  u8 unknown2:7,
     mr_banks:1;
  u8 unknown3;
  u16 mr_index;
  u16 bank_index;
  u16 bank_enable;
  u8 unknown4[5];
  u8 unknown5:6,
     power:2;
  u8 unknown6:4,
     tune_step:4;
  u8 unknown7:6,
     duplex:2;
  u8 unknown8:6,
     tone_mode:2;
  u8 unknown9:2,
     tone:6;
  u8 unknown10;
  u8 unknown11:6,
     mode:2;
  bbcd freq0[4];
  bbcd offset_freq[4];
  u8 unknown12[2];
  char label[16];
  u8 unknown13[6];
  bbcd band_lower[4];
  bbcd band_upper[4];
  bbcd rx_freq[4];
  u8 unknown14[22];
  bbcd freq1[4];
  u8 unknown15[11];
  u8 unknown16:3,
     volume:5;
  u8 unknown17[18];
  u8 active_menu_item;
  u8 checksum;
} vfo_info[6];

#seekto 0x%(dtmadd)04X; // FT-1D:0e4a, FT2D:094a
struct {
  u8 memory[16];
} dtmf[10];

#seekto 0x0EFE;
struct {
  u8 unknown[2];
  u8 name[16];
} bank_info[24];

#seekto 0x154a;
// These "channels" seem to actually be a structure:
//  first five bits are flags
//      0   Unused (1=entry is unused)
//      1   SW Broadcast
//      2   VHF Marine
//      3   WX (weather)
//      4   ? a mode? ?
//  11 bits of index into frequency tables
//
struct {
    u16 channel[100];
} bank_members[24];
"""

MEM_SLOT = """
struct memslot {
  u8 unknown0:2,
     mode_alt:1,  // mode for FTM-3200D
     clock_shift:1,
     unknown1:4;
  u8 mode:2,
     duplex:2,
     tune_step:4;
  bbcd freq[3];
  u8 power:2,
     digmode:2,   // 0=Analog, 1=AMS, 2=DN, 3=VW
     tone_mode:4;
  u16 charsetbits;
  char label[16];
  bbcd offset[3];
  u8 unknown5:2,
     tone:6;
  u8 unknown6:1,
     dcs:7;
  u16 unknown7;
  u8 unknown8:2,
     att:1,
     autostep:1,
     automode:1,
     unknown9:3;
};

struct flagslot {
  u8 nosubvfo:1,
     unknown:3,
     pskip:1,
     skip:1,
     used:1,
     valid:1;
};
"""

MEM_FORMAT = """
#seekto 0x2D4A;
struct memslot memory[%(memnum)d];
struct memslot Skip[99];
struct memslot PMS[100];
#seekto 0x10ca;
struct memslot Home[11];
#seekto 0x280A;
struct flagslot flag[%(memnum)d];
struct flagslot flagskp[99];
struct flagslot flagPMS[100];
"""

YAESU_PRESET_MEMORIES = """
struct memslot  sw_chan[100];    // only 88 defined
struct memslot  marine[100];     // 89; 57 non-empty, but a gap in indices
struct memslot  wx_chan[10];
"""

MEM_APRS_FORMAT = """
#seekto 0xbeca;
struct {
  u8 rx_baud;
  u8 custom_symbol;
  struct {
    char callsign[6];
    u8 ssid;
  } my_callsign;
  u8 unknown3:4,
     selected_position_comment:4;
  u8 unknown4;
  u8 set_time_manually:1,
     tx_interval_beacon:1,
     ring_beacon:1,
     ring_msg:1,
     aprs_mute:1,
     unknown6:1,
     tx_smartbeacon:1,
     af_dual:1;
  u8 unknown7:1,
     aprs_units_wind_mph:1,
     aprs_units_rain_inch:1,
     aprs_units_temperature_f:1,
     aprs_units_altitude_ft:1,
     unknown8:1,
     aprs_units_distance_m:1,
     aprs_units_position_mmss:1;
  u8 unknown9:6,
     aprs_units_speed:2;
  u8 unknown11:1,
     filter_other:1,
     filter_status:1,
     filter_item:1,
     filter_object:1,
     filter_weather:1,
     filter_position:1,
     filter_mic_e:1;
  u8 unknown12;
  u8 unknown13;
  u8 unknown14;
  u8 unknown15:7,
     latitude_sign:1;
  u8 latitude_degree;
  u8 latitude_minute;
  u8 latitude_second;
  u8 unknown16:7,
     longitude_sign:1;
  u8 longitude_degree;
  u8 longitude_minute;
  u8 longitude_second;
  u8 unknown17:4,
     selected_position:4;
  u8 unknown18:5,
     selected_beacon_status_txt:3;
  u8 unknown19:4,
     beacon_interval:4;
  u8 unknowni21:4,
       tx_delay:4;
  u8 unknown21b:6,
     gps_units_altitude_ft:1,
     gps_units_position_sss:1;
  u8 unknown20:6,
     gps_units_speed:2;
  u8 unknown21c[4];
  struct {
    struct {
      char callsign[6];
      u8 ssid;
    } entry[8];
  } digi_path_7;
  u8 unknown22[18];
  struct {
    char padded_string[16];
  } message_macro[7];
  u8 unknown23:5,
     selected_msg_group:3;
  u8 unknown24;
  struct {
    char padded_string[9];
  } msg_group[8];
  u8 unknown25;
  u8 unknown25a:2,
     timezone:6;
  u8 unknown25b[2];
  u8 active_smartbeaconing;
  struct {
    u8 low_speed_mph;
    u8 high_speed_mph;
    u8 slow_rate_min;
    u8 fast_rate_sec;
    u8 turn_angle;
    u8 turn_slop;
    u8 turn_time_sec;
  } smartbeaconing_profile[3];
  u8 unknown26:2,
     flash_msg:6;
  u8 unknown27:2,
     flash_grp:6;
  u8 unknown28:2,
     flash_bln:6;
  u8 selected_digi_path;
  struct {
    struct {
      char callsign[6];
      u8 ssid;
    } entry[2];
  } digi_path_3_6[4];
  u8 unknown30:6,
     selected_my_symbol:2;
  u8 unknown31[3];
  u8 unknown32:2,
     vibrate_msg:6;
  u8 unknown33:2,
     vibrate_grp:6;
  u8 unknown34:2,
     vibrate_bln:6;
} aprs;

#seekto 0xc26a;
struct {
  char padded_string[60];
} aprs_beacon_status_txt[5];

#seekto 0xFECA;
struct {
  bbcd date[3];
  bbcd time[2];
  u8 sequence;
  u8 unknown1;
  u8 unknown2;
  char sender_callsign[9];
  u8 data_type;
  u8 yeasu_data_type;
  u8 unknown4:1,
     callsign_is_ascii:1,
     unknown5:6;
  u8 unknown6;
  u16 pkt_len;
  u8 unknown7;
  u16 in_use;
  u16 unknown8;
  u16 unknown9;
  u16 unknown10;
} aprs_beacon_meta[60];

#seekto 0x1064A;
struct {
  char dst_callsign[9];
  char path[30];
  u16 flags;
  u8 separator;
  char body[134];
} aprs_beacon_pkt[60];

#seekto 0x137c4;
struct {
  u8 flag;
  char dst_callsign[6];
  u8 dst_callsign_ssid;
  char path_and_body[66];
  u8 unknown[70];
} aprs_message_pkt[60];
"""

MEM_BACKTRACK_FORMAT = """
#seekto 0xdf06;
struct {
  u8 status; // 01 full 08 empty
  u8 reserved0; // 00
  bbcd year; // 17
  bbcd mon; // 06
  bbcd day; // 01
  u8 reserved1; // 06
  bbcd hour; // 21
  bbcd min; // xx
  u8 reserved2; // 00
  u8 reserved3; // 00
  char NShemi[1];
  char lat[3];
  char lat_min[2];
  char lat_dec_sec[4];
  char WEhemi[1];
  char lon[3];
  char lon_min[2];
  char lon_dec_sec[4];
} backtrack[3];

"""

MEM_GM_FORMAT = """
#seekto 0x04ba;
struct {
    u8 unknown:3,
        scan_resume:5;          // 52 SCN.RSM
    u8 unknown1:3,
       dw_resume_interval:5;       // 22 DW RSM
    u8 unknown2;
    u8 unknown3:3,
        apo:5;                  // 02 APO
    u8 unknown4:6,
        gm_ring:2;              // 24 GM RNG
    u8 temp_cf;               // Placeholder as not found
    } first_settings;

#seekto 0x04c0;
struct {
    u8 unknown1:5,
        beep_level:3;           // 05 BEP.LVL
    u8 unknown2:6,
        beep_select:2;          // 04 BEEP
    } beep_settings;

#seekto 0x04ed;
struct {
    u8 unknown1:1,
       unknown2:1,
       unknown3:1,
       unknown4:1,
       unknown5:1,
       unknown6:1,
       unknown7:1,
       unknown8:1;
     } test_bit_field;

#seekto 0x0ced0;
struct {
    char callsign[10];              // 63 MYCALL
    u16 charset;                    // character set ID
    } my_call;

#seekto 0xCF30;
struct {
    u8 unknown0;
    u8 unknown1;
    u8 unknown2;
    u8 unknown3;
    u8 unknown4;
    u8 unknown5;
    u8 unknown6;
    u8 digital_popup;              // 15 DIG.POP
    } digital_settings_more;

#seekto 0xCF7C;
struct {
    u8 unknown0:6,
       ams_tx_mode:2;              // AMS TX Mode
    u8 unknown1;
    u8 unknown2:7,
       standby_beep:1;             // 07 BEP.STB
    u8 unknown3;
    u8 unknown4:6,
       gm_ring:2;                  // 24 GM RNG
    u8 unknown5;
    u8 rx_dg_id;                   // RX DG-ID
    u8 tx_dg_id;                   // TX DG-ID
    u8 unknown6:7,
       vw_mode:1;                  // 16 DIG VW
    u8 unknown7;
    } digital_settings;

#seekto 0x1d6d3;
struct {
    char message[32];
    } GM[10];

#seekto 0x1ddca;
struct {
    struct {
        char name[16];
    } Category[5];
    struct {
        struct {
            char ID[5];             // ASCII numerals
            char name[16];          // blank-fill
            u8 unknown[3];
        } Rooms[20];
    } RoomsPerCategory[5];
} WiresX_settings;
"""

MEM_CHECKSUM_FORMAT = """
#seekto 0x1FDC9;
u8 checksum;
"""

TMODES = ["", "Tone", "TSQL", "DTCS"]
DUPLEX = ["", "-", "+", "split"]
MODES = ["FM", "AM", "WFM"]
STEPS = [5.0, 6.25, 8.33, 10.0, 12.5, 15.0, 20.0, 25.0, 50.0, 100.0, 9.0]
SKIPS = ["", "S", "P"]
FT1_DTMF_CHARS = list("0123456789ABCD*#-")

CHARSET = [f"{int(x)}" for x in range(0, 10)] + \
    [chr(x) for x in range(ord("A"), ord("Z") + 1)] + \
    [" ", ] + \
    [chr(x) for x in range(ord("a"), ord("z") + 1)] + \
    list(".,:;*#_-/&()@!?^ ") + list("\x00" * 100)

POWER_LEVELS = [chirp_common.PowerLevel("Hi", watts=5.00),
                chirp_common.PowerLevel("L3", watts=2.50),
                chirp_common.PowerLevel("L2", watts=1.00),
                chirp_common.PowerLevel("L1", watts=0.05)]
# All the Memory attributes that are fit to make immutable
MEM_ATTRS = [i for i in dir(chirp_common.Memory)
             if not i.startswith('__') and
             i not in ['immutable', 'number', 'empty']]
SKIPNAMES = [f"Skip{i}" for i in range(901, 1000)]
PMSNAMES = [f"{c}{i}" for i in range(1, 51) for c in ['L', 'U']]
HOMENAMES = ["AM", "SW", "50MHz", "FM", "Air", "144MHz", "174MHz",
             "Info1", "430MHz", "470MHz", "Info2"]
WXNAMES = [f"WX{i + 1}" for i in range(0, 10)]
VHFNAMES = [f"Marine{i + 1}" for i in range(0, 88)]
SWNAMES = [f"SWL{i + 1}" for i in range(0, 89)]
YAESUNAMES = WXNAMES + VHFNAMES + SWNAMES

ALLNAMES = SKIPNAMES + PMSNAMES + HOMENAMES + YAESUNAMES
# list of (array name, (list of memories in that array))
# array names must match names of memories defined for radio
YAESUSPECIALS = [
    ("wx_chan", WXNAMES),
    ("marine", VHFNAMES),
    ("sw_chan", SWNAMES),
    ]
SPECIALS = [
    ("Skip", SKIPNAMES),
    ("PMS", PMSNAMES),
    ("Home", HOMENAMES),
    ]
if SPECIALS_IN_BANKS:
    SPECIALS += YAESUSPECIALS
# Band edges are integer Hz.
# From FT2DR manual, p36, for VFO A
# VFO B skips first 4 and last band.
VALID_BANDS = [
    (510000, 1790000),
    (1800000, 30990000),
    (30000000, 75990000),
    (76000000, 107990000),
    (108000000, 136990000),
    (137000000, 173990000),
    (174000000, 221950000),
    (222000000, 419990000),
    (420000000, 774990000),
    (470000000, 773990000),
    (810010000, 999000000)
]

YAESU_PRESETS = {
     # index: ('Name', Frequency, 'Mode', 'Duplex', Offset, 'Comment'),
     'SWL1': ('VOA', 6030000, 'AM', '', 0, 'USA'),
     'SWL2': ('VOA', 6160000, 'AM', '', 0, 'USA'),
     'SWL3': ('VOA', 9760000, 'AM', '', 0, 'USA'),
     'SWL4': ('VOA', 11965000, 'AM', '', 0, 'USA'),
     'SWL5': ('Canada', 9555000, 'AM', '', 0, ''),
     'SWL6': ('Canada', 9660000, 'AM', '', 0, ''),
     'SWL7': ('Canada', 11715000, 'AM', '', 0, ''),
     'SWL8': ('Canada', 11955000, 'AM', '', 0, ''),
     'SWL9': ('BBC', 6195000, 'AM', '', 0, 'UK'),
     'SWL10': ('BBC', 9410000, 'AM', '', 0, 'UK'),
     'SWL11': ('BBC', 12095000, 'AM', '', 0, 'UK'),
     'SWL12': ('BBC', 15310000, 'AM', '', 0, 'UK'),
     'SWL13': ('France', 6090000, 'AM', '', 0, ''),
     'SWL14': ('France', 9790000, 'AM', '', 0, ''),
     'SWL15': ('France', 11670000, 'AM', '', 0, ''),
     'SWL16': ('France', 15195000, 'AM', '', 0, ''),
     'SWL17': ('DW', 6000000, 'AM', '', 0, 'Germany'),
     'SWL18': ('DW', 6075000, 'AM', '', 0, 'Germany'),
     'SWL19': ('DW', 9650000, 'AM', '', 0, 'Germany'),
     'SWL20': ('DW', 9735000, 'AM', '', 0, 'Germany'),
     'SWL21': ('Italy', 5990000, 'AM', '', 0, ''),
     'SWL22': ('Italy', 9575000, 'AM', '', 0, ''),
     'SWL23': ('Italy', 9675000, 'AM', '', 0, ''),
     'SWL24': ('Italy', 17780000, 'AM', '', 0, ''),
     'SWL25': ('Turkey', 7170000, 'AM', '', 0, ''),
     'SWL26': ('Turkey', 7270000, 'AM', '', 0, ''),
     'SWL27': ('Turkey', 9560000, 'AM', '', 0, ''),
     'SWL28': ('Turkey', 11690000, 'AM', '', 0, ''),
     'SWL29': ('Vatican', 9660000, 'AM', '', 0, ''),
     'SWL30': ('Vatican', 11625000, 'AM', '', 0, ''),
     'SWL31': ('Vatican', 11830000, 'AM', '', 0, ''),
     'SWL32': ('Vatican', 15235000, 'AM', '', 0, ''),
     'SWL33': ('Nederland', 5955000, 'AM', '', 0, ''),
     'SWL34': ('Nederland', 6020000, 'AM', '', 0, ''),
     'SWL35': ('Nederland', 9895000, 'AM', '', 0, ''),
     'SWL36': ('Nederland', 11655000, 'AM', '', 0, ''),
     'SWL37': ('Czech', 5985000, 'AM', '', 0, ''),
     'SWL38': ('Czech', 6105000, 'AM', '', 0, ''),
     'SWL39': ('Czech', 9455000, 'AM', '', 0, ''),
     'SWL40': ('Czech', 11860000, 'AM', '', 0, ''),
     'SWL41': ('Portugal', 9780000, 'AM', '', 0, ''),
     'SWL42': ('Portugal', 11630000, 'AM', '', 0, ''),
     'SWL43': ('Portugal', 15550000, 'AM', '', 0, ''),
     'SWL44': ('Portugal', 21655000, 'AM', '', 0, ''),
     'SWL45': ('Spain', 9650000, 'AM', '', 0, ''),
     'SWL46': ('Spain', 11880000, 'AM', '', 0, ''),
     'SWL47': ('Spain', 11910000, 'AM', '', 0, ''),
     'SWL48': ('Spain', 15290000, 'AM', '', 0, ''),
     'SWL49': ('NIKKEI', 6055000, 'AM', '', 0, 'Japan'),
     'SWL50': ('Norway', 7315000, 'AM', '', 0, ''),
     'SWL51': ('Norway', 9590000, 'AM', '', 0, ''),
     'SWL52': ('Norway', 9925000, 'AM', '', 0, ''),
     'SWL53': ('Norway', 9985000, 'AM', '', 0, ''),
     'SWL54': ('Sweden', 6065000, 'AM', '', 0, ''),
     'SWL55': ('Sweden', 9490000, 'AM', '', 0, ''),
     'SWL56': ('Sweden', 15240000, 'AM', '', 0, ''),
     'SWL57': ('Sweden', 17505000, 'AM', '', 0, ''),
     'SWL58': ('Finland', 6120000, 'AM', '', 0, ''),
     'SWL59': ('Finland', 9560000, 'AM', '', 0, ''),
     'SWL60': ('Finland', 11755000, 'AM', '', 0, ''),
     'SWL61': ('Finland', 15400000, 'AM', '', 0, ''),
     'SWL62': ('Russia', 5920000, 'AM', '', 0, ''),
     'SWL63': ('Russia', 5940000, 'AM', '', 0, ''),
     'SWL64': ('Russia', 7200000, 'AM', '', 0, ''),
     'SWL65': ('Russia', 12030000, 'AM', '', 0, ''),
     'SWL66': ('Israel', 7465000, 'AM', '', 0, ''),
     'SWL67': ('Israel', 11585000, 'AM', '', 0, ''),
     'SWL68': ('Israel', 15615000, 'AM', '', 0, ''),
     'SWL69': ('Israel', 17535000, 'AM', '', 0, ''),
     'SWL70': ('India', 6045000, 'AM', '', 0, ''),
     'SWL71': ('India', 9595000, 'AM', '', 0, ''),
     'SWL72': ('India', 1162000, 'AM', '', 0, ''),
     'SWL73': ('India', 15020000, 'AM', '', 0, ''),
     'SWL74': ('China', 7190000, 'AM', '', 0, ''),
     'SWL75': ('China', 7405000, 'AM', '', 0, ''),
     'SWL76': ('China', 9785000, 'AM', '', 0, ''),
     'SWL77': ('China', 11685000, 'AM', '', 0, ''),
     'SWL78': ('Korea', 6135000, 'AM', '', 0, ''),
     'SWL79': ('Korea', 7275000, 'AM', '', 0, ''),
     'SWL80': ('Korea', 9570000, 'AM', '', 0, ''),
     'SWL81': ('Korea', 13670000, 'AM', '', 0, ''),
     'SWL82': ('Japan', 6165000, 'AM', '', 0, ''),
     'SWL83': ('Japan', 7200000, 'AM', '', 0, ''),
     'SWL84': ('Japan', 9750000, 'AM', '', 0, ''),
     'SWL85': ('Japan', 11860000, 'AM', '', 0, ''),
     'SWL86': ('Australia', 5995000, 'AM', '', 0, ''),
     'SWL87': ('Australia', 9580000, 'AM', '', 0, ''),
     'SWL88': ('Australia', 9660000, 'AM', '', 0, ''),
     'SWL89': ('Australia', 12080000, 'AM', '', 0, ''),
     'Marine1': ('SEA 01', 156050000, 'FM', '-', 4600000,
                 'Port Operations and Comm'),
     'Marine2': ('VHF 2', 156100000, 'FM', '-', 4600000, ''),
     'Marine3': ('VHF 3', 156150000, 'FM', '-', 4600000, ''),
     'Marine4': ('VHF 4', 156200000, 'FM', '-', 4600000, ''),
     'Marine5': ('SEA 05', 156250000, 'FM', '-', 4600000,
                 'Port Operations.  VTS in'),
     'Marine6': ('SEA 06', 156300000, 'FM', '', 0, 'Intership Safety'),
     'Marine7': ('SEA 07', 156350000, 'FM', '-', 4600000, 'Commercial'),
     'Marine8': ('SEA 08', 156400000, 'FM', '', 0, 'Commercial (Intership on'),
     'Marine9': ('SEA 09', 156450000, 'FM', '', 0, 'Boater Calling.  Commerc'),
     'Marine10': ('SEA 10', 156500000, 'FM', '', 0, 'Commercial'),
     'Marine11': ('SEA 11', 156550000, 'FM', '', 0,
                  'Commercial.  VTS in sele'),
     'Marine12': ('SEA 12', 156600000, 'FM', '', 0,
                  'Port Operations.  VTS in'),
     'Marine13': ('SEA 13 Guard', 156650000, 'FM', '', 0,
                  'Intership Navigation Saf'),
     'Marine14': ('SEA 14', 156700000, 'FM', '', 0,
                  'Port Operations.  VTS in'),
     'Marine15': ('SEA 15', 156750000, 'FM', '', 0,
                  'Environmental (Receive o'),
     'Marine16': ('SEA 16 Distress', 156800000, 'FM', '', 0,
                  'International Distress, '),
     'Marine17': ('SEA 17', 156850000, 'FM', '', 0, 'State Control'),
     'Marine18': ('SEA 18', 156900000, 'FM', '-', 0, 'Commercial'),
     'Marine19': ('SEA 19', 156950000, 'FM', '-', 0, 'Commercial'),
     'Marine20': ('SEA 20', 157000000, 'FM', '-', 0,
                  'Port Operations (duplex)'),
     'Marine21': ('SEA 21', 157050000, 'FM', '-', 0, 'Port Operations'),
     'Marine22': ('SEA 22', 157100000, 'FM', '-', 0,
                  'Coast Guard Liaison and '),
     'Marine23': ('SEA 23', 157150000, 'FM', '-', 0, 'U.S. Government only'),
     'Marine24': ('SEA 24', 157200000, 'FM', '-', 4600000,
                  'Public Correspondence (M'),
     'Marine25': ('SEA 25', 157250000, 'FM', '-', 4600000,
                  'Public Correspondence (M'),
     'Marine26': ('SEA 26', 157300000, 'FM', '-', 4600000,
                  'Public Correspondence (M'),
     'Marine27': ('SEA 27', 157350000, 'FM', '-', 4600000,
                  'Public Correspondence (M'),
     'Marine28': ('SEA 28', 157400000, 'FM', '-', 4600000,
                  'Public Correspondence (M'),
     'Marine60': ('VHF 60', 156025000, 'FM', '-', 4600000, ''),
     'Marine61': ('VHF 61', 156075000, 'FM', '-', 4600000, ''),
     'Marine62': ('VHF 62', 156125000, 'FM', '-', 4600000, ''),
     'Marine63': ('VHF 63', 156175000, 'FM', '-', 4600000, ''),
     'Marine64': ('VHF 64', 156225000, 'FM', '-', 4600000, ''),
     'Marine65': ('SEA 65', 156275000, 'FM', '-', 0, 'Port Operations'),
     'Marine66': ('SEA 66', 156325000, 'FM', '-', 0, 'Port Operations'),
     'Marine67': ('SEA 67', 156375000, 'FM', '', 0,
                  'Commercial.  Used for Br'),
     'Marine68': ('SEA 68', 156425000, 'FM', '', 0,
                  'Non-Commercial-Working C'),
     'Marine69': ('SEA 69', 156475000, 'FM', '', 0, 'Non-Commercial'),
     'Marine70': ('DSC 70', 156525000, 'FM', '', 0,
                  'Digital Selective Callin'),
     'Marine71': ('SEA 71', 156575000, 'FM', '', 0, 'Non-Commercial'),
     'Marine72': ('SEA 72', 156625000, 'FM', '', 0,
                  'Non-Commercial (Intershi'),
     'Marine73': ('SEA 73', 156675000, 'FM', '', 0, 'Port Operations'),
     'Marine74': ('SEA 74', 156725000, 'FM', '', 0, 'Port Operations'),
     'Marine75': ('VHF 75', 156775000, 'FM', '', 0, ''),
     'Marine76': ('VHF 76', 156825000, 'FM', '', 0, ''),
     'Marine77': ('SEA 77', 156875000, 'FM', '', 0, 'Port Operations'),
     'Marine78': ('SEA 78', 156925000, 'FM', '-', 0, 'Non-Commercial'),
     'Marine79': ('SEA 79', 156975000, 'FM', '-', 0, 'Commercial'),
     'Marine80': ('SEA 80', 157025000, 'FM', '-', 0, 'Commercial'),
     'Marine81': ('SEA 81', 157075000, 'FM', '-', 0,
                  'U.S. Government only - E'),
     'Marine82': ('SEA 82', 157125000, 'FM', '-', 0, 'U.S. Government only'),
     'Marine83': ('SEA 83', 157175000, 'FM', '-', 0, 'U.S. Government only'),
     'Marine84': ('SEA 84', 157225000, 'FM', '-', 4600000,
                  'Public Correspondence (M'),
     'Marine85': ('SEA 85', 157275000, 'FM', '-', 4600000,
                  'Public Correspondence (M'),
     'Marine86': ('SEA 86', 157325000, 'FM', '-', 4600000,
                  'Public Correspondence (M'),
     'Marine87': ('SEA 87', 157375000, 'FM', '-', 0,
                  'Public Correspondence (M'),
     'Marine88': ('SEA 88', 157425000, 'FM', '-', 0,
                  'Public Correspondence in'),
     'WX1': ('WX1PA7', 162550000, 'FM', '', 0, ''),
     'WX2': ('WX2PA1', 162400000, 'FM', '', 0, ''),
     'WX3': ('WX3PA4', 162475000, 'FM', '', 0, ''),
     'WX4': ('WX4PA2', 162425000, 'FM', '', 0, ''),
     'WX5': ('WX5PA3', 162450000, 'FM', '', 0, ''),
     'WX6': ('WX6PA5', 162500000, 'FM', '', 0, ''),
     'WX7': ('WX7PA6', 162525000, 'FM', '', 0, ''),
     'WX8': ('WX8', 161650000, 'FM', '', 0, ''),
     'WX9': ('WX9', 161775000, 'FM', '', 0, ''),
     'WX10': ('WX10', 163275000, 'FM', '', 0, ''),
    }


class FT1Bank(chirp_common.NamedBank):
    """A FT1D bank"""

    def get_name(self):
        _bank = self._model._radio._memobj.bank_info[self.index]

        name = ""
        for i in _bank.name:
            if i == 0xFF:
                break
            name += CHARSET[i & 0x7F]
        return name.rstrip()

    def set_name(self, name: str) -> None:
        _bank = self._model._radio._memobj.bank_info[self.index]
        _bank.name = [CHARSET.index(x) for x in name.ljust(16)[:16]]


class FT1BankModel(chirp_common.BankModel,
                   chirp_common.SpecialBankModelInterface):
    """A FT1D bank model
        Now also inheriting ability to display specials in banks
    """
    def __init__(self, radio, name='Banks') -> None:
        super().__init__(radio, name)

        _banks = self._radio._memobj.bank_info
        self._bank_mappings = []
        for index, _bank in enumerate(_banks):
            bank = FT1Bank(self, f"{index}", f"BANK-{index}")
            bank.index = index
            self._bank_mappings.append(bank)

    def get_bankable_specials(self) -> list:
        """ Returns list of SPECIALS to be displayed in Banks
        """
        _ = [name for s in YAESUSPECIALS for name in s[1]]
        # DAR print(f"Get_bankable_specials: {_}")
        return _

    def get_num_mappings(self) -> int:
        return len(self._bank_mappings)

    def get_mappings(self):
        return self._bank_mappings

    def _channel_numbers_in_bank(self, bank) -> set:
        _bank_used = self._radio._memobj.bank_used[bank.index]
        if _bank_used.in_use == 0xFFFF:
            return set()

        _members = self._radio._memobj.bank_members[bank.index]
        return \
            set([int(ch) + 1 for ch in _members.channel if ch != 0xFFFF])

    def update_vfo(self) -> None:
        """ Uses banks to set the VFO A and B """
        chosen_bank = [None, None]
        chosen_mr = [None, None]

        flags = self._radio._memobj.flag

        # Find a suitable bank and MR for VFO A and B.
        for bank in self.get_mappings():
            for channel in self._channel_numbers_in_bank(bank):
                chosen_bank[0] = bank.index
                chosen_mr[0] = channel
                if channel & 0x7000 != 0:
                    # Ignore preset channels without comment DAR
                    break
                if not flags[channel].nosubvfo:
                    chosen_bank[1] = bank.index
                    chosen_mr[1] = channel
                    break
            if chosen_bank[1]:
                break

        for vfo_index in (0, 1):
            # 3 VFO info structs are stored as 3 pairs of (master, backup)
            _i = vfo_index * 2
            vfo = self._radio._memobj.vfo_info[_i]
            vfo_bak = self._radio._memobj.vfo_info[_i + 1]

            if vfo.checksum != vfo_bak.checksum:
                LOG.warning("VFO settings are inconsistent with backup")
            else:
                if ((chosen_bank[vfo_index] is None) and \
                        (vfo.bank_index != 0xFFFF)):
                    LOG.info(f"Disabling banks for VFO {vfo_index}")
                    vfo.bank_index = 0xFFFF
                    vfo.mr_index = 0xFFFF
                    vfo.bank_enable = 0xFFFF
                elif ((chosen_bank[vfo_index] is not None) and
                      (vfo.bank_index == 0xFFFF)):
                    LOG.info(f"Enabling banks for VFO {vfo_index}")
                    vfo.bank_index = chosen_bank[vfo_index]
                    vfo.mr_index = chosen_mr[vfo_index]
                    vfo.bank_enable = 0x0000
                vfo_bak.bank_index = vfo.bank_index
                vfo_bak.mr_index = vfo.mr_index
                vfo_bak.bank_enable = vfo.bank_enable

    def _update_bank_with_channel_numbers(self, bank, channels_in_bank):
        """ Side-effects in bank_members array """
        _members = self._radio._memobj.bank_members[bank.index]
        if len(channels_in_bank) > len(_members.channel):
            raise InvalidValueError(
                f"Too many entries in bank {bank.index}")

        empty = 0
        for index, channel_number in enumerate(sorted(channels_in_bank)):
            _members.channel[index] = channel_number - 1
            if channel_number & 0x7000 != 0:
                LOG.warning(
                    f"Bank {bank.index} uses Yaesu preset frequency"
                    f"id={channel_number:04x}."
                    f"Chirp cannot see or change that entry.")
            empty = index + 1
        for index in range(empty, len(_members.channel)):
            _members.channel[index] = 0xFFFF

    def add_memory_to_mapping(self, memory, bank) -> None:
        """ Side-effects stored in _bank_used """
        # DAR print("\nadd_memory_to_mapping: ", repr(memory), repr(bank))
        channels_in_bank = self._channel_numbers_in_bank(bank)
        channels_in_bank.add(memory.number)
        self._update_bank_with_channel_numbers(bank, channels_in_bank)

        _bank_used = self._radio._memobj.bank_used[bank.index]
        _bank_used.in_use = 0x06

        # TODO do we need VFO updates here?
        # self.update_vfo()

    def remove_memory_from_mapping(self, memory, bank) -> None:
        """ Side-effects occur in bank_members and bank_used arrays """
        channels_in_bank = self._channel_numbers_in_bank(bank)
        print("\nremove_memory_from_mapping: ",
              memory, memory.number, channels_in_bank)
        try:
            channels_in_bank.remove(memory.number)
        except KeyError:
            raise Exception(
                f"Memory {memory.number} is not in bank {bank}s." \
                f" Cannot remove it")
        self._update_bank_with_channel_numbers(bank, channels_in_bank)

        if not channels_in_bank:
            _bank_used = self._radio._memobj.bank_used[bank.index]
            _bank_used.in_use = 0xFFFF

        # TODO do we need VFO updates here?
        # self.update_vfo()

    def get_mapping_memories(self, bank) -> list:
        memories = []
        for channel in self._channel_numbers_in_bank(bank):
            memories.append(self._radio.get_memory(channel))

        return memories

    def get_memory_mappings(self, memory: int | str) -> list:
        banks = []
        for bank in self.get_mappings():
            if memory.number in self._channel_numbers_in_bank(bank):
                banks.append(bank)

        return banks


# Note: other radios like FTM3200Radio subclass this radio
@directory.register
class FT1Radio(yaesu_clone.YaesuCloneModeRadio):
    """Yaesu FT1DR"""
    BAUD_RATE = 38400
    VENDOR = "Yaesu"
    MODEL = "FT-1D"
    VARIANT = "R"
    FORMATS = [directory.register_format('FT1D ADMS-6', '*.ft1d')]
    class_specials = SPECIALS
    _model = b"AH44M"
    _memsize = 130507
    _block_lengths = [10, 130497]
    _block_size = 32
    MAX_MEM_SLOT = 900
    _mem_params = {
         "memnum": 900,            # size of memories array
         "flgnum": 900,            # size of flags array
         "dtmadd": 0xe4a,          # location of DTMF storage
    }
    _has_vibrate = False
    _has_af_dual = True
    _adms_ext = '.ft1d'

    _SG_RE = re.compile(r"(?P<sign>[-+NESW]?)(?P<d>[\d]+)[\s\.,]*"
                        r"(?P<m>[\d]*)[\s\']*(?P<s>[\d]*)")

    _RX_BAUD = ("off", "1200 baud", "9600 baud")
    _TX_DELAY = ("100ms", "150ms", "200ms", "250ms", "300ms",
                 "400ms", "500ms", "750ms", "1000ms")
    _WIND_UNITS = ("m/s", "mph")
    _RAIN_UNITS = ("mm", "inch")
    _TEMP_UNITS = ("C", "F")
    _ALT_UNITS = ("m", "ft")
    _DIST_UNITS = ("km", "mile")
    _POS_UNITS = ("dd.mmmm'", "dd mm'ss\"")
    _SPEED_UNITS = ("km/h", "knot", "mph")
    _TIME_SOURCE = ("manual", "GPS")
    _TZ = ("-13:00", "-13:30", "-12:00", "-12:30", "-11:00", "-11:30",
           "-10:00", "-10:30", "-09:00", "-09:30", "-08:00", "-08:30",
           "-07:00", "-07:30", "-06:00", "-06:30", "-05:00", "-05:30",
           "-04:00", "-04:30", "-03:00", "-03:30", "-02:00", "-02:30",
           "-01:00", "-01:30", "-00:00", "-00:30", "+01:00", "+01:30",
           "+02:00", "+02:30", "+03:00", "+03:30", "+04:00", "+04:30",
           "+05:00", "+05:30", "+06:00", "+06:30", "+07:00", "+07:30",
           "+08:00", "+08:30", "+09:00", "+09:30", "+10:00", "+10:30",
           "+11:00", "+11:30")
    _BEACON_TYPE = ("Off", "Interval", "SmartBeaconing")
    _SMARTBEACON_PROFILE = ("Off", "Type 1", "Type 2", "Type 3")
    _BEACON_INT = ("30s", "1m", "2m", "3m", "5m", "10m", "15m",
                   "20m", "30m", "60m")
    _DIGI_PATHS = ("OFF", "WIDE1-1", "WIDE1-1, WIDE2-1", "Digi Path 4",
                   "Digi Path 5", "Digi Path 6", "Digi Path 7", "Digi Path 8")
    _MSG_GROUP_NAMES = ("Message Group 1", "Message Group 2",
                        "Message Group 3", "Message Group 4",
                        "Message Group 5", "Message Group 6",
                        "Message Group 7", "Message Group 8")
    _POSITIONS = ("GPS", "Manual Latitude/Longitude",
                  "Manual Latitude/Longitude", "P1", "P2", "P3", "P4",
                  "P5", "P6", "P7", "P8", "P9")
    _FLASH = ("OFF", "2 seconds", "4 seconds", "6 seconds", "8 seconds",
              "10 seconds", "20 seconds", "30 seconds", "60 seconds",
              "CONTINUOUS", "every 2 seconds", "every 3 seconds",
              "every 4 seconds", "every 5 seconds", "every 6 seconds",
              "every 7 seconds", "every 8 seconds", "every 9 seconds",
              "every 10 seconds", "every 20 seconds", "every 30 seconds",
              "every 40 seconds", "every 50 seconds", "every minute",
              "every 2 minutes", "every 3 minutes", "every 4 minutes",
              "every 5 minutes", "every 6 minutes", "every 7 minutes",
              "every 8 minutes", "every 9 minutes", "every 10 minutes")
    _BEEP_SELECT = ("Off", "Key+Scan", "Key")
    _SQUELCH = ["%d" % x for x in range(0, 16)]
    _VOLUME = ["%d" % x for x in range(0, 33)]
    _OPENING_MESSAGE = ("Off", "DC", "Message", "Normal")
    _SCAN_RESUME = ["%.1fs" % (0.5 * x) for x in range(4, 21)] + \
                   ["Busy", "Hold"]
    _SCAN_RESTART = ["%.1fs" % (0.1 * x) for x in range(1, 10)] + \
                    ["%.1fs" % (0.5 * x) for x in range(2, 21)]
    _LAMP_KEY = ["Key %d sec" % x
                 for x in range(2, 11)] + ["Continuous", "OFF"]
    _LCD_CONTRAST = ["Level %d" % x for x in range(1, 16)]
    _LCD_DIMMER = ["Level %d" % x for x in range(1, 7)]
    _TOT_TIME = ["Off"] + ["%.1f min" % (0.5 * x) for x in range(1, 21)]
    _OFF_ON = ("Off", "On")
    _VOL_MODE = ("Normal", "Auto Back")
    _DTMF_MODE = ("Manual", "Auto")
    _DTMF_SPEED = ("50ms", "100ms")
    _DTMF_DELAY = ("50ms", "250ms", "450ms", "750ms", "1000ms")
    _MY_SYMBOL = ("/[ Person", "/b Bike", "/> Car", "User selected")
    _BACKTRACK_STATUS = ("Valid", "Invalid")
    _NS_HEMI = ("N", "S")
    _WE_HEMI = ("W", "E")
    _APRS_HIGH_SPEED_MAX = 70
    _MIC_GAIN = ("Level %d" % i for i in range(1, 10))
    _AMS_TX_MODE = ("TX Auto", "TX DIGITAL", "TX FM")
    _VW_MODE = ("On", "Off")
    _DIG_POP_UP = ("Off", "2sec", "4sec", "6sec", "8sec", "10sec",
                   "20sec", "30sec", "60sec", "Continuous")
    _STANDBY_BEEP = ("On", "Off")
    _ON_OFF = ("On", "Off")
    _TEMP_CF = ("Centigrade", "Fahrenheit")
    _APO_SELECT = ("Off", "0.5H", "1.0H", "1.5H", "2.0H", "2.5H",
                   "3.0H", "3.5H", "4.0H", "4.5H", "5.0H",
                   "5.5H", "6.0H", "6.5H", "7.0H", "7.5H", "8.0H",
                   "8.5H", "9.0H", "9.5H", "10.0H", "10.5H",
                   "11.0H", "11.5H", "12.0H")
    _MONI_TCALL = ("Monitor", "Tone-CALL")
    _HOME_REV = ("Home", "Reverse")
    _LOCK = ("KEY", "DIAL", "Key+Dial", "PTT", "Key+PTT", "Dial+PTT", "ALL")
    _PTT_DELAY = ("Off", "20 ms", "50 ms", "100 ms", "200 ms")
    _BEEP_LEVEL = ("Level %i" % i for i in range(1, 7))
    _SET_MODE = ("Level %i" % i for i in range(1, 8))
    _RX_SAVE = ("OFF", "0.2s", ".3s", ".4s", ".5s", ".6s",
                ".7s", ".8s", ".9s", "1.0s", "1.5s",
                "2.0s", "2.5s", "3.0s", "3.5s", "4.0s", "4.5s",
                "5.0s", "5.5s", "6.0s", "6.5s", "7.0s",
                "7.5s", "8.0s", "8.5s", "9.0s", "10.0s", "15s",
                "20s", "25s", "30s", "35s", "40s", "45s", "50s", "55s",
                "60s")
    _VFO_MODE = ("ALL", "BAND")
    _VFO_SCAN_MODE = ("BAND", "ALL")
    _MEMORY_SCAN_MODE = ("BAND", "ALL")

    _RX_BAUD = ("off", "1200 baud", "9600 baud")
    _TX_DELAY = ("100ms", "150ms", "200ms", "250ms", "300ms",
                 "400ms", "500ms", "750ms", "1000ms")
    _WIND_UNITS = ("m/s", "mph")
    _RAIN_UNITS = ("mm", "inch")
    _TEMP_UNITS = ("C", "F")
    _ALT_UNITS = ("m", "ft")
    _DIST_UNITS = ("km", "mile")
    _POS_UNITS = ("dd.mmmm'", "dd mm'ss\"")
    _SPEED_UNITS = ("km/h", "knot", "mph")
    _TIME_SOURCE = ("manual", "GPS")
    _TZ = ("-13:00", "-13:30", "-12:00", "-12:30", "-11:00", "-11:30",
           "-10:00", "-10:30", "-09:00", "-09:30", "-08:00", "-08:30",
           "-07:00", "-07:30", "-06:00", "-06:30", "-05:00", "-05:30",
           "-04:00", "-04:30", "-03:00", "-03:30", "-02:00", "-02:30",
           "-01:00", "-01:30", "-00:00", "-00:30", "+01:00", "+01:30",
           "+02:00", "+02:30", "+03:00", "+03:30", "+04:00", "+04:30",
           "+05:00", "+05:30", "+06:00", "+06:30", "+07:00", "+07:30",
           "+08:00", "+08:30", "+09:00", "+09:30", "+10:00", "+10:30",
           "+11:00", "+11:30")
    _BEACON_TYPE = ("Off", "Interval", "SmartBeaconing")
    _SMARTBEACON_PROFILE = ("Off", "Type 1", "Type 2", "Type 3")
    _BEACON_INT = ("30s", "1m", "2m", "3m", "5m", "10m", "15m",
                   "20m", "30m", "60m")
    _DIGI_PATHS = ("OFF", "WIDE1-1", "WIDE1-1, WIDE2-1", "Digi Path 4",
                   "Digi Path 5", "Digi Path 6", "Digi Path 7", "Digi Path 8")
    _MSG_GROUP_NAMES = ("Message Group 1", "Message Group 2",
                        "Message Group 3", "Message Group 4",
                        "Message Group 5", "Message Group 6",
                        "Message Group 7", "Message Group 8")
    _POSITIONS = ("GPS", "Manual Latitude/Longitude",
                  "Manual Latitude/Longitude", "P1", "P2", "P3", "P4",
                  "P5", "P6", "P7", "P8", "P9")
    _FLASH = ("OFF", "2 seconds", "4 seconds", "6 seconds", "8 seconds",
              "10 seconds", "20 seconds", "30 seconds", "60 seconds",
              "CONTINUOUS", "every 2 seconds", "every 3 seconds",
              "every 4 seconds", "every 5 seconds", "every 6 seconds",
              "every 7 seconds", "every 8 seconds", "every 9 seconds",
              "every 10 seconds", "every 20 seconds", "every 30 seconds",
              "every 40 seconds", "every 50 seconds", "every minute",
              "every 2 minutes", "every 3 minutes", "every 4 minutes",
              "every 5 minutes", "every 6 minutes", "every 7 minutes",
              "every 8 minutes", "every 9 minutes", "every 10 minutes")
    _BEEP_SELECT = ("Off", "Key+Scan", "Key")
    _SQUELCH = ["%d" % x for x in range(0, 16)]
    _VOLUME = ["%d" % x for x in range(0, 33)]
    _DG_ID = ["%d" % x for x in range(0, 100)]
    _GM_RING = ("OFF", "IN RING", "ALWAYS")
    _GM_INTERVAL = ("LONG", "NORMAL", "OFF")

    _MYCALL_CHR_SET = list(string.ascii_uppercase) + \
        list(string.digits) + ['-', '/']



    @classmethod
    def match_model(self, filedata, filename):
        if filename.endswith(self._adms_ext):
            return True
        else:
            return super().match_model(filedata, filename)

    @classmethod
    def get_prompts(self) -> chirp_common.RadioPrompts:
        rp = chirp_common.RadioPrompts()
        rp.pre_download = _(
            "1. Turn radio off.\n"
            "2. Connect cable to DATA terminal.\n"
            "3. Press and hold in the [F] key while turning the radio on\n"
            "     (\"CLONE\" will appear on the display).\n"
            "4. <b>After clicking OK</b>, press the [BAND] key to send"
            " image.\n")
        rp.pre_upload = _(
            "1. Turn radio off.\n"
            "2. Connect cable to DATA terminal.\n"
            "3. Press and hold in the [F] key while turning the radio on\n"
            "     (\"CLONE\" will appear on the display).\n"
            "4. Press the [Dx] key (\"-WAIT-\" will appear on the LCD).\n")
        return rp

    def process_mmap(self) -> None:
        """ Populate the radio's memory structures from the raw memory image.
            The structures are defined in the x_FORMAT text
            The bitmap is in self._mmap
            The structures are left in self._memobj

            Do the same for the Yaesu presets, except bitmap=self._presets.
         """
        mem_format = MEM_SETTINGS_FORMAT + \
            MEM_SLOT + MEM_FORMAT + \
            MEM_APRS_FORMAT + \
            MEM_GM_FORMAT + MEM_BACKTRACK_FORMAT + \
            MEM_CHECKSUM_FORMAT
        self._memobj = bitwise.parse(mem_format % self._mem_params, self._mmap)
        mem_format = MEM_SLOT + YAESU_PRESET_MEMORIES
        self._presets = bitwise.parse(mem_format, b'\x00' * 32 * 210)

    def get_features(self) -> chirp_common.RadioFeatures:
        rf = chirp_common.RadioFeatures()
        rf.has_dtcs_polarity = False
        rf.valid_modes = list(MODES) + ['NFM', 'DN']
        rf.valid_tmodes = list(TMODES)
        rf.valid_duplexes = list(DUPLEX)
        rf.valid_tuning_steps = [x for x in STEPS if x]
        rf.valid_bands = [(500000, 999900000)]
        rf.valid_skips = SKIPS
        rf.valid_power_levels = POWER_LEVELS
        rf.valid_characters = "".join(CHARSET)
        rf.valid_name_length = 16
        rf.memory_bounds = (1, 900)
        rf.can_odd_split = True
        rf.has_ctone = False
        rf.has_bank_names = True
        rf.has_settings = True
        rf.valid_special_chans = [name for s in SPECIALS for name in s[1]]
        return rf

    def get_raw_memory(self, number):
        return "\n".join([repr(self._memobj.memory[number - 1]),
                          repr(self._memobj.flag[number - 1])])

    def _checksums(self):
        return [yaesu_clone.YaesuChecksum(0x064A, 0x06C8),
                yaesu_clone.YaesuChecksum(0x06CA, 0x0748),
                yaesu_clone.YaesuChecksum(0x074A, 0x07C8),
                yaesu_clone.YaesuChecksum(0x07CA, 0x0848),
                yaesu_clone.YaesuChecksum(0x0000, 0x1FDC9)]

    @staticmethod
    def _add_ff_pad(val, length):
        return val.ljust(length, b"\xFF")[:length]

    @classmethod
    def _strip_ff_pads(self, messages):
        result = []
        for msg_text in messages:
            result.append(str(msg_text).rstrip("\xFF"))
        return result

#   Called with a "memref" index to CHIRP memory (int or str)
#   and optionally with a "extref" extended name.
#   Find and return the corresponding memobj
#   Returns:
#       Corresponding radio- or Yaesu preset-memory object
#       Corresponding radio flag structure (if any)
#       index into the specific memory object structure (int) ndx
#       overall index into memory & specials (int) num
#       an indicator of the specific radio object structure (str)
    def slotloc(self, memref: int | str, extref: str = None):
        array = None
        num = memref
        ename = ""
        mstr = isinstance(memref, str)
        specials = ALLNAMES
        extr = False
        if extref is not None:
            extr = extref in specials
        if mstr or extr:        # named special?
            ename = memref
            num = self.MAX_MEM_SLOT + 1
            # num = -1
            sname = memref if mstr else extref
            # Find name of appropriate memory and index into that memory
            for x in self.class_specials:
                try:
                    ndx = x[1].index(sname)
                    array = x[0]
                    break
                except Exception:
                    num += len(x[1])
                    # num -= len(x[1])
            if array is None:
                raise IndexError("Unknown special %s" % memref)
            num += ndx
            # num -= ndx
        elif memref > self.MAX_MEM_SLOT:         # numbered special
            ename = extref
            ndx = memref - (self.MAX_MEM_SLOT + 1)
            # Find name of appropriate memory and index into that memory
            # But assumes specials are in reverse-quantity order
            for x in self.class_specials:
                if ndx < len(x[1]):
                    array = x[0]
                    break
                ndx -= len(x[1])
            if array is None:
                raise IndexError("Unknown memref number %s" % memref)
        else:
            array = "memory"
            ndx = memref - 1
            _flag = self._memobj.flag[ndx]
            _mem = getattr(self._memobj, array)[ndx]
        if array == SPECIALS[0][0]:         # 'Skip'
            _flag = self._memobj.flagskp[ndx]
            _mem = getattr(self._memobj, array)[ndx]
        elif array == SPECIALS[1][0]:       # 'PMS'
            _flag = self._memobj.flagPMS[ndx]
            _mem = getattr(self._memobj, array)[ndx]
        elif array == SPECIALS[2][0]:       # "Home":
            _flag = None
            _mem = getattr(self._memobj, array)[ndx]
        elif array == SPECIALS[3][0]:       # "wx_chan":
            _flag = None
            _mem = getattr(self._presets, array)[ndx]   # Yaesu presets
        elif array == SPECIALS[4][0]:       # "marine":
            _flag = None
            _mem = getattr(self._presets, array)[ndx]
        elif array == SPECIALS[5][0]:       # "sw_chan":
            _flag = None
            _mem = getattr(self._presets, array)[ndx]
        return (_mem, _flag,  ndx, num, array, ename)

    # Build CHIRP version (mem) of radio's memory (_mem)
    def get_memory(self, number: int | str) -> chirp_common.Memory:
        _mem, _flag, ndx, num, array, ename = self.slotloc(number)
        mem = chirp_common.Memory()
        mem.number = num
        # First play with name/extd_number silliness and some mutability
        if array == "Home":
            mem.empty = False
            mem.extd_number = ename
            mem.name = self._decode_label(_mem)
            mem.immutable += ["empty", "number", "extd_number", "skip"]
        elif array != "memory":
            mem.extd_number = ename
            mem.immutable += ["extd_number"]
        else:
            mem.name = self._decode_label(_mem)

        if _flag is not None:
            mem.skip = _flag.pskip and "P" or _flag.skip and "S" or ""
            mem.empty = False
            if not _flag.used:
                mem.empty = True
            if not _flag.valid:
                mem.empty = True
        else:
            # some radios return 0xff's instead of zeroes
            if _mem.charsetbits == 0xffff and _mem.dcs == 0x7f:
                mem.empty = True

        if mem.empty:
            mem.freq = 0
            mem.offset = 0
            mem.duplex = ""
            mem.power = POWER_LEVELS[0]
            mem.mode = "FM"
            mem.tuning_step = STEPS[0]
        else:
            mem.freq = chirp_common.fix_rounded_step(int(_mem.freq) * 1000)
            mem.offset = int(_mem.offset) * 1000
            mem.rtone = mem.ctone = \
                chirp_common.TONES[min(_mem.tone,
                                   len(chirp_common.TONES))]
            self._get_tmode(mem, _mem)
            if mem.duplex is None:
                mem.duplex = DUPLEX[""]
            else:
                mem.duplex = DUPLEX[_mem.duplex]
            if mem.duplex == "split":
                mem.offset = chirp_common.fix_rounded_step(mem.offset)
            mem.mode = self._decode_mode(_mem)
            mem.dtcs = chirp_common.DTCS_CODES[_mem.dcs]
            mem.tuning_step = STEPS[_mem.tune_step]
            mem.power = self._decode_power_level(_mem)
            self._get_mem_extra(mem, _mem)

        # Presets are all immutable and defined ones are never empty
        if array in [YAESUSPECIALS[_i][0] for _i in range(0, 3)]:
            try:
                mem.name, mem.freq, mem.mode, mem.duplex, \
                    mem.offset, mem.comment = YAESU_PRESETS[number]
                mem.empty = False
            except KeyError:
                mem.empty = True
            mem.immutable += ['name', 'freq', 'mode', 'duplex', 'offset',
                              'comment', 'empty']
        # DAR if mem.number > 1050:
        # DAR    print(f"{mem.number}, {mem.extd_number}: " \
        # DAR          f"{mem.empty}, {mem.name}, {mem.freq}, {mem.mode}, " \
        # DAR          f"{mem.duplex}, {mem.offset}, {mem.comment}")
        return mem

    def _get_mem_extra(self, mem, _mem):
        mem.extra = RadioSettingGroup('Extra', 'extra')

        ams = _mem.digmode == 1
        rs = RadioSetting('ysf_ams', 'AMS mode',
                          RadioSettingValueBoolean(ams))
        mem.extra.append(rs)

    def _set_mem_extra(self, mem, _mem):
        if 'ysf_ams' in mem.extra:
            # We only set AMS if the memory mode is DN. If it is FM,
            # then that takes precedence as "analog-only".
            if mem.mode == 'DN':
                orig = int(_mem.digmode)
                _mem.digmode = int(mem.extra['ysf_ams'].value) and 1 or 2
                LOG.debug('Changed digmode from %i to %i' % (
                    orig, _mem.digmode))

    def _decode_label(self, mem):
        charset = ''.join(CHARSET).ljust(256, '.')
        return str(mem.label).rstrip("\xFF").translate(charset)

    def _encode_label(self, mem):
        label = "".join([chr(CHARSET.index(x))
                         for x in mem.name.rstrip()]).encode()
        return self._add_ff_pad(label, 16)

    def _encode_charsetbits(self, mem):
        # We only speak English here in chirpville
        return 0x0000

    def _decode_power_level(self, mem):
        return POWER_LEVELS[3 - mem.power]

    def _encode_power_level(self, mem):
        if mem.power is None:
            return 3        # Choose lowest power
        else:
            return 3 - POWER_LEVELS.index(mem.power)

    def _decode_mode(self, mem):
        mode = MODES[min(mem.mode, len(MODES))]
        if mode == 'FM' and int(mem.digmode):
            # DN mode is FM with a digital flag. Since digmode can be AMS or
            # DN, either will be called 'DN' in chirp
            return 'DN'
        if mode == 'FM' and int(mem.mode_alt):
            return 'NFM'
        else:
            return mode

    def _encode_mode(self, mem):
        mode = mem.mode
        if mode == 'NFM':
            # Narrow is handled by a separate flag
            mode = 'FM'
        elif mode == 'DN':
            # DN mode is FM with a digital flag
            mode = 'FM'
        return MODES.index(mode)

    def _get_tmode(self, mem, _mem):
        mem.tmode = TMODES[min(_mem.tone_mode, len(TMODES))]

    def _set_tmode(self, _mem, mem):
        _mem.tone_mode = TMODES.index(mem.tmode)

    def _set_mode(self, _mem, mem):
        _mem.mode_alt = mem.mode == 'NFM'
        if mem.mode == 'DN' and int(_mem.digmode) == 0:
            # If we are going to DN mode, default to AMS
            LOG.debug('New mode DN, setting AMS')
            if 'ysf_ams' not in mem.extra:
                _mem.digmode = 1
            else:
                # If we have the extra setting, use that, since it will be
                # applied at the end of set_memory() and we want it to apply
                # the right value.
                mem.extra['ysf_ams'].value = True
        elif mem.mode == 'FM' and int(_mem.digmode):
            # If we are going back to FM, that means analog-only, so AMS
            # is disabled
            LOG.debug('New mode FM, disabling AMS')
            _mem.digmode = 0
        _mem.mode = self._encode_mode(mem)
        bm = self.get_bank_model()
        for bank in bm.get_memory_mappings(mem):
            bm.remove_memory_from_mapping(mem, bank)

    def _debank(self, mem):
        bm = self.get_bank_model()
        for bank in bm.get_memory_mappings(mem):
            bm.remove_memory_from_mapping(mem, bank)

    def validate_memory(self, mem):
        # Only check the home registers for appropriate bands
        msgs = super().validate_memory(mem)
        ndx = mem.number - ALLNAMES.index("AM") - self.MAX_MEM_SLOT - 1
        if 10 >= ndx >= 0:
            f = VALID_BANDS[ndx]
            if not f[0] < mem.freq < f[1]:
                msgs.append(chirp_common.ValidationError(
                            "Frequency outside of band for Home%2d" %
                            (ndx + 1)))
        return msgs

    # Modify radio's memory (_mem) corresponding to CHIRP version at 'mem'
    def set_memory(self, mem: int | str) -> None:
        _mem, flag, ndx, num, regtype, ename = self.slotloc(mem.number,
                                                            mem.extd_number)
        # Ignore all attempts to change presets
        if regtype in (SPECIALS[3][0], SPECIALS[4][0], SPECIALS[5][0]):
            return

        if mem.empty:
            self._wipe_memory(_mem)
            if flag is not None:
                flag.used = False
            return
        _mem.power = self._encode_power_level(mem)
        _mem.tone = chirp_common.TONES.index(mem.rtone)
        self._set_tmode(_mem, mem)
        _mem.dcs = chirp_common.DTCS_CODES.index(mem.dtcs)
        _mem.tune_step = STEPS.index(mem.tuning_step)
        # duplex "off" is equivalent to "" and may show up in tox test.
        if mem.duplex is None:
            _mem.duplex = DUPLEX.index("")
        else:
            _mem.duplex = DUPLEX.index(mem.duplex)
        self._set_mode(_mem, mem)
        if flag is not None:
            if mem.freq < 30000000 or \
                    (mem.freq > 88000000 and mem.freq < 108000000) or \
                    mem.freq > 580000000:
                flag.nosubvfo = True     # Masked from VFO B
            else:
                flag.nosubvfo = False    # Available in both VFOs
        if regtype != "Home":
            self._debank(mem)
            ndx = num - 1
            flag.used = not mem.empty
            flag.valid = True
            flag.skip = mem.skip == "S"
            flag.pskip = mem.skip == "P"
        freq = mem.freq
        _mem.freq = int(freq / 1000)
        _mem.offset = int(mem.offset / 1000)
        _mem.label = self._encode_label(mem)
        _mem.charsetbits = self._encode_charsetbits(mem)
        self._set_mem_extra(mem, _mem)
        return

    @classmethod
    def _wipe_memory(self, mem):
        mem.set_raw("\x00" * (mem.size() // 8))
        mem.unknown1 = 0x05

    def get_bank_model(self):
        return FT1BankModel(self)

    @classmethod
    def _digi_path_to_str(self, path):
        path_cmp = []
        for entry in path.entry:
            callsign = str(entry.callsign).rstrip("\xFF")
            if not callsign:
                break
            path_cmp.append("%s-%d" % (callsign, entry.ssid))
        return ",".join(path_cmp)

    @staticmethod
    def _latlong_sanity(sign, l_d, l_m, l_s, is_lat):
        if sign not in (0, 1):
            sign = 0
        if is_lat:
            d_max = 90
        else:
            d_max = 180
        if l_d < 0 or l_d > d_max:
            l_d = 0
            l_m = 0
            l_s = 0
        if l_m < 0 or l_m > 60:
            l_m = 0
            l_s = 0
        if l_s < 0 or l_s > 60:
            l_s = 0
        return sign, l_d, l_m, l_s

    @classmethod
    def _latlong_to_str(self, sign, l_d, l_m, l_s, is_lat,
                        to_sexigesimal=True):
        sign, l_d, l_m, l_s = self._latlong_sanity(
            sign, l_d, l_m, l_s, is_lat)
        mult = sign and -1 or 1
        if to_sexigesimal:
            return f"{mult * l_d},{l_m}'{l_s}\""
        _r = f"{(mult * l_d + (l_m / 60.0) + (l_s / (60.0 * 60.0))):0.5f}"
        return _r

    @classmethod
    def _str_to_latlong(self, lat_long, is_lat):
        sign = 0
        result = [0, 0, 0]

        lat_long = lat_long.strip()

        if not lat_long:
            return 1, 0, 0, 0

        try:
            # DD.MMMMM is the simple case, try that first.
            val = float(lat_long)
            if val < 0:
                sign = 1
            val = abs(val)
            result[0] = int(val)
            result[1] = int(val * 60) % 60
            result[2] = int(val * 3600) % 60
        except ValueError:
            # Try DD MM'SS" if DD.MMMMM failed.
            match = self._SG_RE.match(lat_long.strip())
            if match:
                if match.group("sign") and (match.group("sign") in "SE-"):
                    sign = 1
                else:
                    sign = 0
                if match.group("d"):
                    result[0] = int(match.group("d"))
                if match.group("m"):
                    result[1] = int(match.group("m"))
                if match.group("s"):
                    result[2] = int(match.group("s"))
            elif len(lat_long) > 4:
                raise Exception("Lat/Long should be DD MM'SS\" or DD.MMMMM")

        return self._latlong_sanity(sign, result[0], result[1], result[2],
                                    is_lat)

    def _get_aprs_settings(self) -> RadioSettingGroup:
        menu = RadioSettingGroup("aprs_top", "APRS")
        menu.append(self._get_aprs_general_settings())
        menu.append(self._get_aprs_rx_settings())
        menu.append(self._get_aprs_tx_settings())
        menu.append(self._get_aprs_smartbeacon())
        menu.append(self._get_aprs_msgs())
        menu.append(self._get_aprs_beacons())
        return menu

    def _get_aprs_general_settings(self) -> RadioSettingGroup:
        menu = RadioSettingGroup("aprs_general", "APRS General")
        aprs = self._memobj.aprs

        val = RadioSettingValueString(
            0, 6, str(aprs.my_callsign.callsign).rstrip("\xFF"))
        rs = RadioSetting("aprs.my_callsign.callsign", "My Callsign", val)
        rs.set_apply_callback(self.apply_callsign, aprs.my_callsign)
        menu.append(rs)

        val = RadioSettingValueList(
            chirp_common.APRS_SSID,
            current_index=aprs.my_callsign.ssid)
        rs = RadioSetting("aprs.my_callsign.ssid", "My SSID", val)
        menu.append(rs)

        val = RadioSettingValueList(self._MY_SYMBOL,
                                    current_index=aprs.selected_my_symbol)
        rs = RadioSetting("aprs.selected_my_symbol", "My Symbol", val)
        menu.append(rs)

        symbols = list(chirp_common.APRS_SYMBOLS)
        selected = aprs.custom_symbol
        if aprs.custom_symbol >= len(chirp_common.APRS_SYMBOLS):
            symbols.append(f"{aprs.custom_symbol}")
            selected = len(symbols) - 1
        val = RadioSettingValueList(symbols, current_index=selected)
        rs = RadioSetting("aprs.custom_symbol_text", "User Selected Symbol",
                          val)
        rs.set_apply_callback(self.apply_custom_symbol, aprs)
        menu.append(rs)

        val = RadioSettingValueList(
            chirp_common.APRS_POSITION_COMMENT,
            current_index=aprs.selected_position_comment)
        rs = RadioSetting("aprs.selected_position_comment", "Position Comment",
                          val)
        menu.append(rs)

        latitude = self._latlong_to_str(aprs.latitude_sign,
                                        aprs.latitude_degree,
                                        aprs.latitude_minute,
                                        aprs.latitude_second,
                                        True, aprs.aprs_units_position_mmss)
        longitude = self._latlong_to_str(aprs.longitude_sign,
                                         aprs.longitude_degree,
                                         aprs.longitude_minute,
                                         aprs.longitude_second,
                                         False, aprs.aprs_units_position_mmss)

        # TODO: Rebuild this when aprs_units_position_mmss changes.
        # TODO: Rebuild this when latitude/longitude change.
        # TODO: Add saved positions p1 - p10 to memory map.
        position_str = list(self._POSITIONS)
        # position_str[1] = "%s %s" % (latitude, longitude)
        # position_str[2] = "%s %s" % (latitude, longitude)
        val = RadioSettingValueList(position_str,
                                    current_index=aprs.selected_position)
        rs = RadioSetting("aprs.selected_position", "My Position", val)
        menu.append(rs)

        val = RadioSettingValueString(0, 10, latitude)
        rs = RadioSetting("latitude", "Manual Latitude", val)
        rs.set_apply_callback(self.apply_lat_long, aprs)
        menu.append(rs)

        val = RadioSettingValueString(0, 11, longitude)
        rs = RadioSetting("longitude", "Manual Longitude", val)
        rs.set_apply_callback(self.apply_lat_long, aprs)
        menu.append(rs)

        val = RadioSettingValueList(
            self._TIME_SOURCE, current_index=aprs.set_time_manually)
        rs = RadioSetting("aprs.set_time_manually", "Time Source", val)
        menu.append(rs)

        val = RadioSettingValueList(self._TZ, current_index=aprs.timezone)
        rs = RadioSetting("aprs.timezone", "Timezone", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._SPEED_UNITS, current_index=aprs.aprs_units_speed)
        rs = RadioSetting("aprs.aprs_units_speed", "APRS Speed Units", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._SPEED_UNITS, current_index=aprs.gps_units_speed)
        rs = RadioSetting("aprs.gps_units_speed", "GPS Speed Units", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._ALT_UNITS, current_index=aprs.aprs_units_altitude_ft)
        rs = RadioSetting(
            "aprs.aprs_units_altitude_ft", "APRS Altitude Units", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._ALT_UNITS, current_index=aprs.gps_units_altitude_ft)
        rs = RadioSetting("aprs.gps_units_altitude_ft", "GPS Altitude Units",
                          val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._POS_UNITS, current_index=aprs.aprs_units_position_mmss)
        rs = RadioSetting("aprs.aprs_units_position_mmss",
                          "APRS Position Format", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._POS_UNITS, current_index=aprs.gps_units_position_sss)
        rs = RadioSetting("aprs.gps_units_position_sss",
                          "GPS Position Format", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._DIST_UNITS, current_index=aprs.aprs_units_distance_m)
        rs = RadioSetting(
            "aprs.aprs_units_distance_m", "APRS Distance Units", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._WIND_UNITS, current_index=aprs.aprs_units_wind_mph)
        rs = RadioSetting(
            "aprs.aprs_units_wind_mph", "APRS Wind Speed Units", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._RAIN_UNITS, current_index=aprs.aprs_units_rain_inch)
        rs = RadioSetting(
            "aprs.aprs_units_rain_inch", "APRS Rain Units", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._TEMP_UNITS, current_index=aprs.aprs_units_temperature_f)
        rs = RadioSetting(
            "aprs.aprs_units_temperature_f", "APRS Temperature Units", val)
        menu.append(rs)

        return menu

    def _get_aprs_msgs(self) -> RadioSettingGroup:
        menu = RadioSettingGroup("aprs_msg", "APRS Messages")
        aprs_msg = self._memobj.aprs_message_pkt

        for index in range(0, 60):
            if aprs_msg[index].flag != 255:
                astring = \
                    str(aprs_msg[index].dst_callsign).partition("\xFF")[0]

                val = RadioSettingValueString(
                    0, 9, chirp_common.sanitize_string(astring) +
                    f"-{aprs_msg[index].dst_callsign_ssid}")
                val.set_mutable(False)
                rs = RadioSetting(
                    "aprs_msg.dst_callsign{index}",
                    "Dst Callsign {index}", val)
                menu.append(rs)

                astring = \
                    str(aprs_msg[index].path_and_body).partition("\xFF")[0]
                val = RadioSettingValueString(
                    0, 66, chirp_common.sanitize_string(astring))
                val.set_mutable(False)
                rs = RadioSetting(
                    f"aprs_msg.path_and_body{index}", "Body", val)
                menu.append(rs)

        return menu

    def _get_aprs_beacons(self) -> RadioSettingGroup:
        menu = RadioSettingGroup("aprs_beacons", "APRS Beacons")
        aprs_beacon = self._memobj.aprs_beacon_pkt
        aprs_meta = self._memobj.aprs_beacon_meta

        for index in range(0, 60):
            # There is probably a more pythonesque way to do this
            scl = int(aprs_meta[index].sender_callsign[0])
            dcl = int(aprs_beacon[index].dst_callsign[0])
            if scl != 255 and scl != 0:  # ignore if empty send call
                callsign = str(aprs_meta[index].sender_callsign).rstrip("\xFF")
                val = RadioSettingValueString(0, 9, callsign)
                val.set_mutable(False)
                rs = RadioSetting(
                    f"aprs_beacon.src_callsign{index}",
                    f"SRC Callsign {index}", val)
                menu.append(rs)

                if dcl != 255 and dcl != 0:   # ignore if empty dest call
                    val = str(aprs_beacon[index].dst_callsign)
                    val = RadioSettingValueString(0, 9, val.rstrip("\xFF"))
                    val.set_mutable(False)
                    rs = RadioSetting(
                        f"aprs_beacon.dst_callsign{index}",
                        f"DST Callsign {index}", val)
                    menu.append(rs)

                date = f"{aprs_meta[index].date[0]:02d}/" \
                       f"{aprs_meta[index].date[1]:02d}/" \
                       f"{aprs_meta[index].date[2]:02d}"
                val = RadioSettingValueString(0, 8, date)
                val.set_mutable(False)
                rs = RadioSetting(f"aprs_beacon.date{index}", "Date", val)
                menu.append(rs)

                time = f"{aprs_meta[index].time[0]:02d}:" \
                       f"{aprs_meta[index].time[1]:02d}"
                val = RadioSettingValueString(0, 5, time)
                val.set_mutable(False)
                rs = RadioSetting(f"aprs_beacon.time{index}", "Time", val)
                menu.append(rs)

                if dcl != 255 and dcl != 0:   # ignore if empty dest call
                    path = str(aprs_beacon[index].path).replace("\x00", " ")
                    path = ''.join(c for c in path
                                   if c in string.printable).strip()
                    path = str(path).replace("\xE0", "*")
                    val = RadioSettingValueString(0, 32, path)
                    val.set_mutable(False)
                    rs = RadioSetting(
                     f"aprs_beacon.path{index}", "Digipath", val)
                    menu.append(rs)

                body = str(aprs_beacon[index].body).rstrip("\xFF")
                checksum = body[-2:]
                body = ''.join(s for s in body[:-2]
                               if s in string.printable).translate(
                                   str.maketrans(
                                       "", "", "\x09\x0a\x0b\x0c\x0d"))
                try:
                    val = RadioSettingValueString(0, 134, body.strip())
                except Exception as e:
                    LOG.error(f"Error in APRS beacon at index {index}")
                    raise e
                val.set_mutable(False)
                rs = RadioSetting(f"aprs_beacon.body{index}", "Body", val)
                menu.append(rs)

        return menu

    def _get_aprs_rx_settings(self) -> RadioSettingGroup:
        menu = RadioSettingGroup("aprs_rx", "APRS Receive")
        aprs = self._memobj.aprs

        val = RadioSettingValueList(
            self._RX_BAUD, current_index=aprs.rx_baud)
        rs = RadioSetting("aprs.rx_baud", "Modem RX", val)
        menu.append(rs)

        val = RadioSettingValueBoolean(aprs.aprs_mute)
        rs = RadioSetting("aprs.aprs_mute", "APRS Mute", val)
        menu.append(rs)

        if self._has_af_dual:
            val = RadioSettingValueBoolean(aprs.af_dual)
            rs = RadioSetting("aprs.af_dual", "AF Dual", val)
            menu.append(rs)

        val = RadioSettingValueBoolean(aprs.ring_msg)
        rs = RadioSetting("aprs.ring_msg", "Ring on Message RX", val)
        menu.append(rs)

        val = RadioSettingValueBoolean(aprs.ring_beacon)
        rs = RadioSetting("aprs.ring_beacon", "Ring on Beacon RX", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._FLASH, current_index=aprs.flash_msg)
        rs = RadioSetting(
            "aprs.flash_msg", "Flash on personal message", val)
        menu.append(rs)

        if self._has_vibrate:
            val = RadioSettingValueList(
                self._FLASH, current_index=aprs.vibrate_msg)
            rs = RadioSetting(
                "aprs.vibrate_msg", "Vibrate on personal message", val)
            menu.append(rs)

        val = RadioSettingValueList(
            self._FLASH[:10], current_index=aprs.flash_bln)
        rs = RadioSetting(
            "aprs.flash_bln", "Flash on bulletin message", val)
        menu.append(rs)

        if self._has_vibrate:
            val = RadioSettingValueList(
                self._FLASH[:10], current_index=aprs.vibrate_bln)
            rs = RadioSetting(
                "aprs.vibrate_bln", "Vibrate on bulletin message", val)
            menu.append(rs)

        val = RadioSettingValueList(
            self._FLASH[:10], current_index=aprs.flash_grp)
        rs = RadioSetting("aprs.flash_grp", "Flash on group message", val)
        menu.append(rs)

        if self._has_vibrate:
            val = RadioSettingValueList(
                self._FLASH[:10], current_index=aprs.vibrate_grp)
            rs = RadioSetting(
                "aprs.vibrate_grp", "Vibrate on group message", val)
            menu.append(rs)

        filter_val = [m.padded_string for m in aprs.msg_group]
        filter_val = self._strip_ff_pads(filter_val)
        for index, filter_text in enumerate(filter_val):
            val = RadioSettingValueString(0, 9, filter_text)
            rs = RadioSetting(f"aprs.msg_group_{index}",
                              f"Message Group {index + 1}", val)
            menu.append(rs)
            rs.set_apply_callback(self.apply_ff_padded_string,
                                  aprs.msg_group[index])
        # TODO: Use filter_val as the list entries and update it on edit.
        val = RadioSettingValueList(
            self._MSG_GROUP_NAMES,
            current_index=aprs.selected_msg_group)
        rs = RadioSetting(
            "aprs.selected_msg_group", "Selected Message Group", val)
        menu.append(rs)

        val = RadioSettingValueBoolean(aprs.filter_mic_e)
        rs = RadioSetting(
            "aprs.filter_mic_e", "Receive Mic-E Beacons", val)
        menu.append(rs)

        val = RadioSettingValueBoolean(aprs.filter_position)
        rs = RadioSetting(
            "aprs.filter_position", "Receive Position Beacons", val)
        menu.append(rs)

        val = RadioSettingValueBoolean(aprs.filter_weather)
        rs = RadioSetting(
            "aprs.filter_weather", "Receive Weather Beacons", val)
        menu.append(rs)

        val = RadioSettingValueBoolean(aprs.filter_object)
        rs = RadioSetting(
            "aprs.filter_object", "Receive Object Beacons", val)
        menu.append(rs)

        val = RadioSettingValueBoolean(aprs.filter_item)
        rs = RadioSetting("aprs.filter_item", "Receive Item Beacons", val)
        menu.append(rs)

        val = RadioSettingValueBoolean(aprs.filter_status)
        rs = RadioSetting(
            "aprs.filter_status", "Receive Status Beacons", val)
        menu.append(rs)

        val = RadioSettingValueBoolean(aprs.filter_other)
        rs = RadioSetting(
            "aprs.filter_other", "Receive Other Beacons", val)
        menu.append(rs)

        return menu

    def _get_aprs_tx_settings(self) -> RadioSettingGroup:
        menu = RadioSettingGroup("aprs_tx", "APRS Transmit")
        aprs = self._memobj.aprs

        beacon_type = (aprs.tx_smartbeacon << 1) | aprs.tx_interval_beacon
        val = RadioSettingValueList(
            self._BEACON_TYPE, current_index=beacon_type)
        rs = RadioSetting("aprs.transmit", "TX Beacons", val)
        rs.set_apply_callback(self.apply_beacon_type, aprs)
        menu.append(rs)

        val = RadioSettingValueList(
            self._TX_DELAY, current_index=aprs.tx_delay)
        rs = RadioSetting("aprs.tx_delay", "TX Delay", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._BEACON_INT, current_index=aprs.beacon_interval)
        rs = RadioSetting("aprs.beacon_interval", "Beacon Interval", val)
        menu.append(rs)

        desc = []
        status = [
            m.padded_string for m in self._memobj.aprs_beacon_status_txt]
        status = self._strip_ff_pads(status)
        for index, msg_text in enumerate(status):
            val = RadioSettingValueString(0, 60, msg_text)
            desc.append(f"Beacon Status Text {index + 1}")
            rs = RadioSetting(
                f"aprs_beacon_status_txt_{index}", desc[-1], val)
            rs.set_apply_callback(
                self.apply_ff_padded_string,
                self._memobj.aprs_beacon_status_txt[index])
            menu.append(rs)
        val = RadioSettingValueList(
            desc, current_index=aprs.selected_beacon_status_txt)
        rs = RadioSetting(
            "aprs.selected_beacon_status_txt", "Beacon Status Text", val)
        menu.append(rs)

        message_macro = [m.padded_string for m in aprs.message_macro]
        message_macro = self._strip_ff_pads(message_macro)
        for index, msg_text in enumerate(message_macro):
            val = RadioSettingValueString(0, 16, msg_text)
            rs = RadioSetting(
                f"aprs.message_macro_{index}",
                f"Message Macro {index + 1}", val)
            rs.set_apply_callback(
                self.apply_ff_padded_string, aprs.message_macro[index])
            menu.append(rs)

        path_str = list(self._DIGI_PATHS)
        path_str[3] = self._digi_path_to_str(aprs.digi_path_3_6[0])
        val = RadioSettingValueString(0, 22, path_str[3])
        rs = RadioSetting(
            "aprs.digi_path_3", "Digi Path 4 (2 entries)", val)
        rs.set_apply_callback(self.apply_digi_path, aprs.digi_path_3_6[0])
        menu.append(rs)

        path_str[4] = self._digi_path_to_str(aprs.digi_path_3_6[1])
        val = RadioSettingValueString(0, 22, path_str[4])
        rs = RadioSetting(
            "aprs.digi_path_4", "Digi Path 5 (2 entries)", val)
        rs.set_apply_callback(self.apply_digi_path, aprs.digi_path_3_6[1])
        menu.append(rs)

        path_str[5] = self._digi_path_to_str(aprs.digi_path_3_6[2])
        val = RadioSettingValueString(0, 22, path_str[5])
        rs = RadioSetting(
            "aprs.digi_path_5", "Digi Path 6 (2 entries)", val)
        rs.set_apply_callback(self.apply_digi_path, aprs.digi_path_3_6[2])
        menu.append(rs)

        path_str[6] = self._digi_path_to_str(aprs.digi_path_3_6[3])
        val = RadioSettingValueString(0, 22, path_str[6])
        rs = RadioSetting(
            "aprs.digi_path_6", "Digi Path 7 (2 entries)", val)
        rs.set_apply_callback(self.apply_digi_path, aprs.digi_path_3_6[3])
        menu.append(rs)

        path_str[7] = self._digi_path_to_str(aprs.digi_path_7)
        val = RadioSettingValueString(0, 88, path_str[7])
        rs = RadioSetting(
            "aprs.digi_path_7", "Digi Path 8 (8 entries)", val)
        rs.set_apply_callback(self.apply_digi_path, aprs.digi_path_7)
        menu.append(rs)

        # Show friendly messages for empty slots rather than blanks.
        # TODO: Rebuild this when digi_path_[34567] change.
        # path_str[3] = path_str[3] or self._DIGI_PATHS[3]
        # path_str[4] = path_str[4] or self._DIGI_PATHS[4]
        # path_str[5] = path_str[5] or self._DIGI_PATHS[5]
        # path_str[6] = path_str[6] or self._DIGI_PATHS[6]
        # path_str[7] = path_str[7] or self._DIGI_PATHS[7]
        path_str[3] = self._DIGI_PATHS[3]
        path_str[4] = self._DIGI_PATHS[4]
        path_str[5] = self._DIGI_PATHS[5]
        path_str[6] = self._DIGI_PATHS[6]
        path_str[7] = self._DIGI_PATHS[7]
        val = RadioSettingValueList(
            path_str, current_index=aprs.selected_digi_path)
        rs = RadioSetting(
            "aprs.selected_digi_path", "Selected Digi Path", val)
        menu.append(rs)

        return menu

    def _get_aprs_smartbeacon(self) -> RadioSettingGroup:
        menu = RadioSettingGroup("aprs_smartbeacon", "APRS SmartBeacon")
        aprs = self._memobj.aprs

        val = RadioSettingValueList(
            self._SMARTBEACON_PROFILE,
            current_index=aprs.active_smartbeaconing)
        rs = RadioSetting(
            "aprs.active_smartbeaconing", "SmartBeacon profile", val)
        menu.append(rs)

        for profile in range(3):
            pfx = f"type {profile + 1}"
            path = f"aprs.smartbeaconing_profile[{profile}]"
            prof = aprs.smartbeaconing_profile[profile]

            low_val = RadioSettingValueInteger(2, 30, prof.low_speed_mph)
            high_val = RadioSettingValueInteger(
                3, self._APRS_HIGH_SPEED_MAX, prof.high_speed_mph)
            low_val.get_max = lambda: min(30, int(high_val.get_value()) - 1)

            rs = RadioSetting(
                f"{path}.low_speed_mph", f"{pfx} Low Speed (mph)", low_val)
            menu.append(rs)

            rs = RadioSetting(
                f"{path}.high_speed_mph",
                f"{pfx} High Speed (mph)", high_val)
            menu.append(rs)

            val = RadioSettingValueInteger(1, 100, prof.slow_rate_min)
            rs = RadioSetting(f"{path}.slow_rate_min",
                              f"{pfx} Slow rate (minutes)", val)
            menu.append(rs)

            val = RadioSettingValueInteger(10, 180, prof.fast_rate_sec)
            rs = RadioSetting(f"{path}.fast_rate_sec",
                              f"{pfx} Fast rate (seconds)", val)
            menu.append(rs)

            val = RadioSettingValueInteger(5, 90, prof.turn_angle)
            rs = RadioSetting(f"{path}.turn_angle",
                              f"{pfx} Turn angle (degrees)", val)
            menu.append(rs)

            val = RadioSettingValueInteger(1, 255, prof.turn_slop)
            rs = RadioSetting(f"{path}.turn_slop",
                              f"{pfx}Turn slop", val)
            menu.append(rs)

            val = RadioSettingValueInteger(5, 180, prof.turn_time_sec)
            rs = RadioSetting(f"{path}.turn_time_sec",
                              f"{pfx} Turn time (seconds)", val)
            menu.append(rs)

        return menu

    def _get_digital_settings(self) -> RadioSettingGroup:
        topmenu = RadioSettingGroup("digital_settings", "Digital")
        menu = RadioSettingGroup("settings", "Digital Modes")
        topmenu.append(menu)
        GMmenu = RadioSettingGroup("first_settings", "Group Monitor(GM)")
        topmenu.append(GMmenu)
        WXmenu = RadioSettingGroup("WiresX_settings", "Wires-X")
        topmenu.append(WXmenu)

        # MYCALL
        mycall = self._memobj.my_call
        mycallstr = str(mycall.callsign).rstrip("\xff").rstrip()
        mycalle = RadioSettingValueString(
            0, 10, mycallstr, False, charset=self._MYCALL_CHR_SET)
        rs = RadioSetting('mycall.callsign',
                          'MYCALL (10 uppercase chars)', mycalle)
        rs.set_apply_callback(self.apply_mycall, mycall)
        menu.append(rs)

        # Short Press AMS button AMS TX Mode
        digital_settings = self._memobj.digital_settings
        val = RadioSettingValueList(
            self._AMS_TX_MODE,
            current_index=digital_settings.ams_tx_mode)
        rs = RadioSetting("digital_settings.ams_tx_mode",
                          "AMS TX Mode", val)
        menu.append(rs)

        # 16 DIG VW  Turn the VW mode selection ON or OFF.
        val = RadioSettingValueList(
            self._VW_MODE,
            current_index=digital_settings.vw_mode)
        rs = RadioSetting("digital_settings.vw_mode", "VW Mode", val)
        menu.append(rs)

        # TX DG-ID Long Press Mode Key, Dial
        val = RadioSettingValueList(
            self._DG_ID, current_index=digital_settings.tx_dg_id)
        rs = RadioSetting("digital_settings.tx_dg_id",
                          "TX DG-ID", val)
        menu.append(rs)

        # RX DG-ID Long Press Mode Key, Mode Key to select, Dial
        val = RadioSettingValueList(
            self._DG_ID, current_index=digital_settings.rx_dg_id)
        rs = RadioSetting("digital_settings.rx_dg_id",
                          "RX DG-ID", val)
        menu.append(rs)

        # 15 DIG.POP    Call sign display pop up time
        digital_settings_more = self._memobj.digital_settings_more

        val = RadioSettingValueList(
            self._DIG_POP_UP,
            current_index=(
                0 if digital_settings_more.digital_popup == 0
                else digital_settings_more.digital_popup - 9))

        rs = RadioSetting("digital_settings_more.digital_popup",
                          "Digital Popup", val)
        rs.set_apply_callback(self.apply_digital_popup,
                              digital_settings_more)
        menu.append(rs)

        # 07  BEP.STB    Standby Beep in the digital C4FM mode. On/Off
        val = RadioSettingValueList(
            self._STANDBY_BEEP,
            current_index=digital_settings.standby_beep)
        rs = RadioSetting("digital_settings.standby_beep",
                          "Standby Beep", val)
        menu.append(rs)

        # GM settings
        # 24 GM RNG Select the beep option
        first_settings = self._memobj.first_settings
        val = RadioSettingValueList(
            self._GM_RING,
            current_index=first_settings.gm_ring)
        rs = RadioSetting("first_settings.gm_ring", "GM Ring", val)
        GMmenu.append(rs)

        # 25 GM INT transmission interval of digital GM info
        scan_settings = self._memobj.scan_settings
        val = RadioSettingValueList(
            self._GM_INTERVAL,
            current_index=scan_settings.gm_interval)
        rs = RadioSetting("scan_settings.gm_interval",
                          "GM Interval", val)
        GMmenu.append(rs)

        m = self._memobj.GM
        for i in range(0, 10):
            cname = f"GM[{i}].message"
            msg = str(m[i].message).rstrip("\xff)")
            val = RadioSettingValueString(0, 32, msg)
            rs = RadioSetting(cname, f"GM Message {i + 1}", val)
            GMmenu.append(rs)

        # WiresX settings
        wxc = self._memobj.WiresX_settings
        for i in range(0, 5):
            cname = f"WiresX_settings.Category[{i + 1}].name"
            c = ''
            for j in range(0, 16):
                s = wxc.Category[i].name[j]
                if int(s) != 0xff:
                    c = c + str(s)
            val = RadioSettingValueString(0, 16, c)
            rs = RadioSetting(cname, f"Category {i + 1}", val)
            rs.set_apply_callback(self.apply_WiresX_category,
                                  wxc.Category[i].name)
            WXmenu.append(rs)

            r = wxc.RoomsPerCategory[i]
            rn = False
            for j in range(0, 20):
                idn = "0"
                if int(r.Rooms[j].ID[1]) != 0xff:
                    idn = r.Rooms[j].ID
                    rn = False
                elif rn:
                    break
                elif j > 0:
                    rn = True
                val = RadioSettingValueInteger(0, 99999, int(str(idn)))
                vname = f"WiresX_settings.RoomsperCategory{i}" \
                    f"Rooms[{j}].ID"
                rs = RadioSetting(
                        vname, f"   Room ID{j + 1} (5 numerals)", val)
                rs.set_apply_callback(
                    self.apply_WiresX_roomid, r.Rooms[j])
                WXmenu.append(rs)
                cn = ''
                for l in range(0, 16):
                    s = r.Rooms[j].name[l]
                    if int(s) != 0xff:
                        cn = cn + str(s)
                val = RadioSettingValueString(0, 16, str(cn))
                cname = f"WiresX_settings.RoomsperCategory{i} " \
                    f"Rooms[{j}].name"
                rs = RadioSetting(
                    cname, f"   Room Name{j+1} (16 chars)", val)
                rs.set_apply_callback(
                    self.apply_WiresX_roomname, r.Rooms[j])
                WXmenu.append(rs)
            pass
        return topmenu

    def apply_WiresX_category(self, setting, obj):
        val = setting.value.get_value()
        setattr(obj, "name", val)

    def apply_WiresX_roomid(self, setting, obj):
        val = setting.value.get_value()
        obj.ID = self.zero_pad(val, 5)

    def apply_WiresX_roomname(self, setting, obj):
        val = setting.value.get_value()
        obj.name = str(val)

    def _get_dtmf_settings(self):
        menu = RadioSettingGroup("dtmf_settings", "DTMF")
        dtmf = self._memobj.scan_settings

        val = RadioSettingValueList(
            self._DTMF_MODE,
            current_index=dtmf.dtmf_mode)
        rs = RadioSetting("scan_settings.dtmf_mode", "DTMF Mode", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._DTMF_SPEED,
            current_index=dtmf.dtmf_speed)
        rs = RadioSetting(
            "scan_settings.dtmf_speed", "DTMF AutoDial Speed", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._DTMF_DELAY,
            current_index=dtmf.dtmf_delay)
        rs = RadioSetting(
            "scan_settings.dtmf_delay", "DTMF AutoDial Delay", val)
        menu.append(rs)

        for i in range(10):
            name = f"dtmf_{i:02d}"
            dtmfsetting = self._memobj.dtmf[i]
            dtmfstr = ""
            for c in dtmfsetting.memory:
                if c == 0xFF:
                    break
                if c < len(FT1_DTMF_CHARS):
                    dtmfstr += FT1_DTMF_CHARS[c]
            dtmfentry = RadioSettingValueString(0, 16, dtmfstr)
            dtmfentry.set_charset(FT1_DTMF_CHARS + list("abcd "))
            rs = RadioSetting(name, name.upper(), dtmfentry)
            rs.set_apply_callback(self.apply_dtmf, i)
            menu.append(rs)

        return menu

    def _get_misc_settings(self) -> RadioSettingGroup:
        menu = RadioSettingGroup("misc_settings", "Misc")
        scan_settings = self._memobj.scan_settings

        val = RadioSettingValueList(
            self._LCD_DIMMER,
            current_index=scan_settings.lcd_dimmer)
        rs = RadioSetting("scan_settings.lcd_dimmer", "LCD Dimmer", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._LCD_CONTRAST,
            current_index=scan_settings.lcd_contrast - 1)
        rs = RadioSetting(
            "scan_settings.lcd_contrast", "LCD Contrast", val)
        rs.set_apply_callback(self.apply_lcd_contrast, scan_settings)
        menu.append(rs)

        val = RadioSettingValueList(
            self._LAMP_KEY,
            current_index=scan_settings.lamp)
        rs = RadioSetting("scan_settings.lamp", "Lamp", val)
        menu.append(rs)

        beep_select = self._memobj.beep_select

        val = RadioSettingValueList(
            self._BEEP_SELECT,
            current_index=beep_select.beep)
        rs = RadioSetting("beep_select.beep", "Beep Select", val)
        menu.append(rs)

        opening_message = self._memobj.opening_message

        val = RadioSettingValueList(
            self._OPENING_MESSAGE,
            current_index=opening_message.flag)
        rs = RadioSetting("opening_message.flag", "Opening Msg Mode",
                          val)
        menu.append(rs)

        rs = self._decode_opening_message(opening_message)
        menu.append(rs)

        return menu

    def _decode_opening_message(self, opening_message) -> RadioSetting:
        msg = ""
        for i in opening_message.message.padded_yaesu:
            if i == 0xFF:
                break
            msg += CHARSET[i & 0x7F]
        val = RadioSettingValueString(0, 16, msg)
        rs = RadioSetting("opening_message.message.padded_yaesu",
                          "Opening Message", val)
        rs.set_apply_callback(self.apply_ff_padded_yaesu,
                              opening_message.message)
        return rs

    def backtrack_ll_validate(
            self, number: int, minl: int, maxl: int) -> bool:
        if str(number).lstrip('0').strip().isdigit() and \
                int(str(number).lstrip('0')) <= maxl and \
                int(str(number).lstrip('0')) >= minl:
            return True

        return False

    @staticmethod
    def zero_pad(number, length):
        """ Applies a leading zero pad of length `length` to `number` """
        return str(number).rjust(length, "0")

    def _get_backtrack_settings(self) -> RadioSettingGroup:

        menu = RadioSettingGroup("backtrack", "Backtrack")
        for i in range(3):
            prefix = ''
            if i == 0:
                prefix = "Star "
            if i == 1:
                prefix = "L1 "
            if i == 2:
                prefix = "L2 "

            bt_idx = f"backtrack[{i}]"

            bt = self._memobj.backtrack[i]

            val = RadioSettingValueList(
                self._BACKTRACK_STATUS,
                current_index=0 if bt.status == 1 else 1)
            rs = RadioSetting(f"{bt_idx}.status", prefix + "status", val)
            rs.set_apply_callback(self.apply_backtrack_status, bt)
            menu.append(rs)

            if bt.status == 1 and int(bt.year) < 100:
                val = RadioSettingValueInteger(0, 99, bt.year)
            else:
                val = RadioSettingValueInteger(0, 99, 0)
            rs = RadioSetting(f"{bt_idx}.year", prefix + "year", val)
            menu.append(rs)

            if bt.status == 1 and int(bt.mon) <= 12:
                val = RadioSettingValueInteger(0, 12, bt.mon)
            else:
                val = RadioSettingValueInteger(0, 12, 0)
            rs = RadioSetting(f"{bt_idx}.mon", prefix + "month", val)
            menu.append(rs)

            if bt.status == 1:
                val = RadioSettingValueInteger(0, 31, bt.day)
            else:
                val = RadioSettingValueInteger(0, 31, 0)
            rs = RadioSetting(f"{bt_idx}.day", prefix + "day", val)
            menu.append(rs)

            if bt.status == 1:
                val = RadioSettingValueInteger(0, 23, bt.hour)
            else:
                val = RadioSettingValueInteger(0, 23, 0)
            rs = RadioSetting(f"{bt_idx}s.hour", prefix + "hour", val)
            menu.append(rs)

            if bt.status == 1:
                val = RadioSettingValueInteger(0, 59, bt.min)
            else:
                val = RadioSettingValueInteger(0, 59, 0)
            rs = RadioSetting(f"{bt_idx}.min", prefix + "min", val)
            menu.append(rs)

            if bt.status == 1 and (str(bt.NShemi) in ('N', 'S')):
                val = RadioSettingValueString(0, 1, str(bt.NShemi))
            else:
                val = RadioSettingValueString(0, 1, ' ')
            rs = RadioSetting(
                f"{bt_idx}.NShemi", prefix + "NS hemisphere", val)
            rs.set_apply_callback(self.apply_NShemi, bt)
            menu.append(rs)

            if bt.status == 1 and self.backtrack_ll_validate(bt.lat, 0, 90):
                val = RadioSettingValueString(
                        0, 3, self.zero_pad(bt.lat, 3))
            else:
                val = RadioSettingValueString(0, 3, '   ')
            rs = RadioSetting(f"{bt_idx}.lat", prefix + "Latitude", val)
            rs.set_apply_callback(self.apply_bt_lat, bt)
            menu.append(rs)

            if bt.status == 1 and \
                    self.backtrack_ll_validate(bt.lat_min, 0, 59):
                val = RadioSettingValueString(
                    0, 2, self.zero_pad(bt.lat_min, 2))
            else:
                val = RadioSettingValueString(0, 2, '  ')
            rs = RadioSetting(
                    f"{bt_idx}.lat_min",
                    prefix + "Latitude Minutes", val)
            rs.set_apply_callback(self.apply_bt_lat_min, bt)
            menu.append(rs)

            if bt.status == 1 and \
                    self.backtrack_ll_validate(bt.lat_dec_sec, 0, 9999):
                val = RadioSettingValueString(
                    0, 4, self.zero_pad(bt.lat_dec_sec, 4))
            else:
                val = RadioSettingValueString(0, 4, '    ')
            rs = RadioSetting(
                    f"{bt_idx}.lat_dec_sec",
                    prefix + "Latitude Decimal Seconds", val)
            rs.set_apply_callback(self.apply_bt_lat_dec_sec, bt)
            menu.append(rs)

            if bt.status == 1 and \
                    (str(bt.WEhemi) == 'W' or str(bt.WEhemi) == 'E'):
                val = RadioSettingValueString(
                    0, 1, str(bt.WEhemi))
            else:
                val = RadioSettingValueString(0, 1, ' ')
            rs = RadioSetting(
                    f"{bt_idx}.WEhemi",
                    prefix + "WE hemisphere", val)
            rs.set_apply_callback(self.apply_WEhemi, bt)
            menu.append(rs)

            if bt.status == 1 and \
                    self.backtrack_ll_validate(bt.lon, 0, 180):
                val = RadioSettingValueString(
                    0, 3, self.zero_pad(bt.lon, 3))
            else:
                val = RadioSettingValueString(0, 3, '   ')
            rs = RadioSetting(f"{bt_idx}.lon", prefix + "Longitude", val)
            rs.set_apply_callback(self.apply_bt_lon, bt)
            menu.append(rs)

            if bt.status == 1 and \
                    self.backtrack_ll_validate(bt.lon_min, 0, 59):
                val = RadioSettingValueString(
                    0, 2, self.zero_pad(bt.lon_min, 2))
            else:
                val = RadioSettingValueString(0, 2, '  ')
            rs = RadioSetting(
                    f"{bt_idx}.lon_min",
                    prefix + "Longitude Minutes", val)
            rs.set_apply_callback(self.apply_bt_lon_min, bt)
            menu.append(rs)

            if bt.status == 1 and \
                    self.backtrack_ll_validate(bt.lon_dec_sec, 0, 9999):
                val = RadioSettingValueString(
                    0, 4, self.zero_pad(bt.lon_dec_sec, 4))
            else:
                val = RadioSettingValueString(0, 4, '    ')
            rs = RadioSetting(
                f"{bt_idx}.lon_dec_sec",
                prefix + "Longitude Decimal Seconds", val)
            rs.set_apply_callback(self.apply_bt_lon_dec_sec, bt)
            menu.append(rs)

        return menu

    def _get_scan_settings(self) -> RadioSettingGroup:
        menu = RadioSettingGroup("scan_settings", "Scan")
        scan_settings = self._memobj.scan_settings

        val = RadioSettingValueList(
            self._VOL_MODE,
            current_index=scan_settings.vol_mode)
        rs = RadioSetting("scan_settings.vol_mode", "Volume Mode", val)
        menu.append(rs)

        vfoa = self._memobj.vfo_info[0]
        val = RadioSettingValueList(
            self._VOLUME,
            current_index=vfoa.volume)
        rs = RadioSetting("vfo_info[0].volume", "VFO A Volume", val)
        rs.set_apply_callback(self.apply_volume, 0)
        menu.append(rs)

        vfob = self._memobj.vfo_info[1]
        val = RadioSettingValueList(
            self._VOLUME,
            current_index=vfob.volume)
        rs = RadioSetting("vfo_info[1].volume", "VFO B Volume", val)
        rs.set_apply_callback(self.apply_volume, 1)
        menu.append(rs)

        squelch = self._memobj.squelch
        val = RadioSettingValueList(
            self._SQUELCH,
            current_index=squelch.vfo_a)
        rs = RadioSetting("squelch.vfo_a", "VFO A Squelch", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._SQUELCH,
            current_index=squelch.vfo_b)
        rs = RadioSetting("squelch.vfo_b", "VFO B Squelch", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._SCAN_RESTART,
            current_index=scan_settings.scan_restart)
        rs = RadioSetting("scan_settings.scan_restart", "Scan Restart", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._SCAN_RESUME,
            current_index=scan_settings.scan_resume)
        rs = RadioSetting("scan_settings.scan_resume", "Scan Resume", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._OFF_ON,
            current_index=scan_settings.busy_led)
        rs = RadioSetting("scan_settings.busy_led", "Busy LED", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._OFF_ON,
            current_index=scan_settings.scan_lamp)
        rs = RadioSetting("scan_settings.scan_lamp", "Scan Lamp", val)
        menu.append(rs)

        val = RadioSettingValueList(
            self._TOT_TIME,
            current_index=scan_settings.tot)
        rs = RadioSetting("scan_settings.tot", "Transmit Timeout (TOT)", val)
        menu.append(rs)

        return menu

    def _get_settings(self) -> RadioSettings:
        top = RadioSettings(
                            self._get_aprs_settings(),
                            self._get_digital_settings(),
                            self._get_dtmf_settings(),
                            self._get_misc_settings(),
                            self._get_scan_settings(),
                            self._get_backtrack_settings())
        return top

    def get_settings(self) -> RadioSettings:
        try:
            return self._get_settings()
        except:
            LOG.error(
                "Failed to parse settings: %s", traceback.format_exc())
            return None

    @staticmethod
    def apply_custom_symbol(setting, obj):
        # Ensure new value falls within known bounds, otherwise leave it as
        # it's a custom value from the radio that's outside our list.
        if setting.value.get_value() in chirp_common.APRS_SYMBOLS:
            setattr(obj, "custom_symbol",
                    chirp_common.APRS_SYMBOLS.index(
                        setting.value.get_value()))

    @classmethod
    def _apply_callsign(self, callsign, obj, default_ssid=None):
        ssid = default_ssid
        dash_index = callsign.find("-")
        if dash_index >= 0:
            ssid = callsign[dash_index + 1:]
            callsign = callsign[:dash_index]
            try:
                ssid = int(ssid) % 16
            except ValueError:
                ssid = default_ssid
        setattr(obj, "callsign", self._add_ff_pad(callsign.encode(), 6))
        if ssid is not None:
            setattr(obj, "ssid", ssid)

    def apply_beacon_type(self, setting, obj):
        beacon_type = str(setting.value.get_value())
        beacon_index = self._BEACON_TYPE.index(beacon_type)
        tx_smartbeacon = beacon_index >> 1
        tx_interval_beacon = beacon_index & 1
        if tx_interval_beacon:
            setattr(obj, "tx_interval_beacon", 1)
            setattr(obj, "tx_smartbeacon", 0)
        elif tx_smartbeacon:
            setattr(obj, "tx_interval_beacon", 0)
            setattr(obj, "tx_smartbeacon", 1)
        else:
            setattr(obj, "tx_interval_beacon", 0)
            setattr(obj, "tx_smartbeacon", 0)

    # digital settings callback routines
    def apply_digital_popup(self, setting, obj):
        rawval = setting.value.get_value()
        val = 0 if self._DIG_POP_UP.index(rawval) == 0 \
            else self._DIG_POP_UP.index(rawval) + 9
        obj.digital_popup = val

    def apply_mycall(self, setting, obj):
        cs = setting.value.get_value()
        if cs[0] in ('-', '/'):
            raise InvalidValueError("First character of"
                                    " call sign can't be - or /:  {0:s}"
                                    .format(cs))
        else:
            obj.callsign = self._add_ff_pad(cs.rstrip(), 10)

    @classmethod
    def apply_callsign(self, setting, obj, default_ssid=None):
        # Uppercase, strip SSID then FF pad to max string length.
        callsign = setting.value.get_value().upper()
        self._apply_callsign(callsign, obj, default_ssid)

    def apply_digi_path(self, setting, obj):
        # Parse and map to aprs.digi_path_4_7[0-3] or aprs.digi_path_8
        # and FF terminate.
        path = str(setting.value.get_value())
        callsigns = [c.strip() for c in path.split(",")]
        for index in range(len(obj.entry)):
            try:
                self._apply_callsign(callsigns[index], obj.entry[index], 0)
            except IndexError:
                self._apply_callsign("", obj.entry[index], 0)
        if len(callsigns) > len(obj.entry):
            raise Exception(
                "This path only supports %d entries" % (index + 1))

    @classmethod
    def apply_ff_padded_string(cls, setting, obj):
        # FF pad.
        val = setting.value.get_value()
        max_len = getattr(obj, "padded_string").size() / 8
        val = str(val).rstrip()
        setattr(obj, "padded_string", cls._add_ff_pad(val, max_len))

    @classmethod
    def apply_lat_long(cls, setting, obj):
        name = setting.get_name()
        is_latitude = name.endswith("latitude")
        lat_long = setting.value.get_value().strip()
        sign, l_d, l_m, l_s = cls._str_to_latlong(lat_long, is_latitude)
        LOG.debug(f"{name}: {sign} {l_d} {l_m} {l_s}")
        setattr(obj, f"{name}_sign", sign)
        setattr(obj, f"{name}_degree", l_d)
        setattr(obj, f"{name}_minute", l_m)
        setattr(obj, f"{name}_second", l_s)

    def set_settings(self, settings):
        _mem = self._memobj
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue
            if not element.changed():
                continue
            try:
                if element.has_apply_callback():
                    LOG.debug("Using apply callback")
                    try:
                        element.run_apply_callback()
                    except NotImplementedError as e:
                        LOG.error("ft1d.set_settings: %s", e)
                    continue

                # Find the object containing setting.
                obj = _mem
                bits = element.get_name().split(".")
                setting = bits[-1]
                for name in bits[:-1]:
                    if name.endswith("]"):
                        name, index = name.split("[")
                        index = int(index[:-1])
                        obj = getattr(obj, name)[index]
                    else:
                        obj = getattr(obj, name)

                try:
                    old_val = getattr(obj, setting)
                    LOG.debug(f"Setting {element.get_name()}({old_val})"
                              f" <= {element.value}")
                    setattr(obj, setting, element.value)
                except AttributeError as e:
                    LOG.error(f"Setting {element.get_name()} is not"
                              f" in the memory map: {e}")
            except Exception:
                LOG.debug(element.get_name())
                raise

    def apply_ff_padded_yaesu(self, setting, obj):
        # FF pad yaesus custom string format.
        rawval = setting.value.get_value()
        max_len = getattr(obj, "padded_yaesu").size() / 8
        rawval = str(rawval).rstrip()
        val = [CHARSET.index(x) for x in rawval]
        for x in range(len(val), max_len):
            val.append(0xFF)
        obj.padded_yaesu = val

    def apply_volume(self, setting, vfo):
        val = setting.value.get_value()
        self._memobj.vfo_info[(vfo * 2)].volume = val
        self._memobj.vfo_info[(vfo * 2) + 1].volume = val

    def apply_lcd_contrast(self, setting, obj):
        rawval = setting.value.get_value()
        val = self._LCD_CONTRAST.index(rawval) + 1
        obj.lcd_contrast = val

    def apply_dtmf(self, setting, i):
        rawval = setting.value.get_value().upper().rstrip()
        val = [FT1_DTMF_CHARS.index(x) for x in rawval]
        for x in range(len(val), 16):
            val.append(0xFF)
        self._memobj.dtmf[i].memory = val

    def apply_backtrack_status(self, setting, obj):
        status = setting.value.get_value()

        if status == 'Valid':
            val = 1
        else:
            val = 8
        setattr(obj, "status", val)

    def apply_NShemi(self, setting, obj):
        hemi = setting.value.get_value().upper()

        if hemi not in ('N', 'S'):
            hemi = ' '
        setattr(obj, "NShemi", hemi)

    def apply_WEhemi(self, setting, obj):
        hemi = setting.value.get_value().upper()

        if hemi not in ('W', 'E'):
            hemi = ' '
        setattr(obj, "WEhemi", hemi)

    def backtrack_zero_pad(self, val: str, leng: int = 0) -> str:
        """ Dummy, doesn't seem to make much difference. DAR """
        return val

    def apply_bt_lat(self, setting, obj):
        val = setting.value.get_value()
        val = self.backtrack_zero_pad(val, 3)

        setattr(obj, "lat", val)

    def apply_bt_lat_min(self, setting, obj):
        val = setting.value.get_value()
        val = self.backtrack_zero_pad(val, 2)

        setattr(obj, "lat_min", val)

    def apply_bt_lat_dec_sec(self, setting, obj):
        val = setting.value.get_value()
        val = self.backtrack_zero_pad(val, 4)

        setattr(obj, "lat_dec_sec", val)

    def apply_bt_lon(self, setting, obj):
        val = setting.value.get_value()
        val = self.backtrack_zero_pad(val, 3)

        setattr(obj, "lon", val)

    def apply_bt_lon_min(self, setting, obj):
        val = setting.value.get_value()
        val = self.backtrack_zero_pad(val, 2)

        setattr(obj, "lon_min", val)

    def apply_bt_lon_dec_sec(self, setting, obj):
        val = setting.value.get_value()
        val = self.backtrack_zero_pad(val, 4)

        setattr(obj, "lon_dec_sec", val)

    def load_mmap(self, filename):
        if filename.lower().endswith(self._adms_ext):
            with open(filename, 'rb') as f:
                self._adms_header = f.read(0x16)
                LOG.debug(f'ADMS Header:\n{util.hexprint(self._adms_header)}')
                self._mmap = memmap.MemoryMapBytes(self._model + f.read())
                LOG.info('Loaded ADMS file')
            self.process_mmap()
        else:
            chirp_common.CloneModeRadio.load_mmap(self, filename)

    def save_mmap(self, filename):
        if filename.lower().endswith(self._adms_ext):
            if not hasattr(self, '_adms_header'):
                raise Exception('Unable to save .img to {self._adms_ext}')
            with open(filename, 'wb') as f:
                f.write(self._adms_header)
                f.write(self._mmap.get_packed()[5:])
                LOG.info('Wrote file')
        else:
            chirp_common.CloneModeRadio.save_mmap(self, filename)
