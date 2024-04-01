"""
NMEA Protocol Set payload definitions

THESE ARE THE PAYLOAD DEFINITIONS FOR _SET_ MESSAGES _TO_ THE RECEIVER
(e.g. Configuration commands).

NB: Attribute names must be unique within each message id.
NB: Avoid reserved names 'msgID', 'talker', 'payload', 'checksum'.

NB: Repeating groups must be defined as a tuple thus
    'group': ('numr', {dict})
    where
    - 'numr' is either:
       a) an integer representing a fixed number of repeats e.g 32
       b) a string representing the name of a preceding attribute
          containing the number of repeats e.g. 'numCh'
       c) 'None' for an indeterminate repeating group
          (only one such group is permitted per message type)
    - {dict} is the nested dictionary containing the repeating
      attributes

Created on 4 Mar Sep 2021

While the NMEA 0183 © protocol is proprietary, the information here
has been collated from public domain sources.

:author: semuadmin
"""

from pynmeagps.nmeatypes_core import (
    CH,
    DE,
    DT,
    HX,
    IN,
    LA,
    LN,
    ST,
    TM,
)

NMEA_PAYLOADS_SET = {
    # *********************************************
    # STANDARD MESSAGES
    # *********************************************
    # No standard SET messages that I'm aware of
    # *********************************************
    # GARMIN PROPRIETARY MESSAGES
    # *********************************************
    "GRMI": {  # sensor initialisation information
        "lat": LA,
        "NS": CH,
        "lon": LN,
        "EW": CH,
        "date": DT,
        "time": TM,
        "rcvr_cmd": CH,
    },
    "GRMC": {  # sensor configuration information
        "fix": CH,
        "alt": DE,
        "dtm": ST,
        "smAxis": DE,
        "iffac": DE,
        "xecc": DE,
        "yecc": DE,
        "zecc": DE,
        "diff": CH,
        "baudRate": IN,
        "vfilt": IN,
        "reserved1": ST,
        "reserved2": ST,
        "drtime": IN,
    },
    "GRMC1": {  # additional sensor configuration information
        "nmeatim": IN,
        "bphase": IN,
        "autopos": IN,
        "dgpsfr": DE,
        "dgpsbr": IN,
        "dgpssc": IN,
        "nmeaver": IN,
        "dgpsmod": CH,
        "pwrsave": CH,
        "attran": IN,
        "autopwr": IN,
        "extpwr": IN,
    },
    "GRMO": {  # output sentence enable
        "msgId": ST,
        "tgtmode": IN,
    },
    "GRMW": {  # additional waypoint information
        "wptId": ST,
        "alt": DE,
        "symnum": HX,
        "comment": ST,
    },
    # *********************************************
    # U-BLOX PROPRIETARY MESSAGES
    # *********************************************
    "UBX40": {  # set message rates per port
        "msgId": ST,  # '40'
        "id": IN,
        "rddc": IN,  # I2C
        "rus1": IN,  # UART1
        "rus2": IN,  # UART2
        "rusb": IN,  # USB
        "rspi": IN,  # SPI
        "reserved": IN,
    },
    "UBX41": {  # configure port protocols
        "msgId": ST,  # '41'
        "portId": IN,
        "inProto": HX,
        "outProto": HX,
        "baudRate": IN,
        "autobauding": IN,
    },
    # *********************************************
    # MARITIME ROBOTICS PROPRIETARY MESSAGES
    # *********************************************
    "MARABT": {},
    # For this command, since Otter USV can't provide a force in the y direction, this value isn't used/set
    "MARMAN": {  # If the norm of the vector made by the below 3 values is >1 then it should be normalized to 1
        "force_x": DE, # range [-1,1]
        "force_y": DE, # range [-1,1]
        "torque_z": DE, # range [-1,1]
    },
    "MARCRS": {
        "course": DE,
        "speed": DE,
    },
    "MARLEG": {
        "lat0": LA, # lat of first point
        "lat0_dir": CH,
        "lon0": LN, # lon of first point
        "lon0_dir": CH,
        "lat1": LA, # lat of second point
        "lat1_dir": CH,
        "lon1": LN, # lon of second point
        "lon1_dir": CH,
        "speed": DE,
    },
    "MARSTA": {
        "lat": LA,
        "lat_dir": CH,
        "lon": LN,
        "lon_dir": CH,
        "speed": DE,
    },
    "MARPRM": {
        "lookahead_distance": DE, # (leg mode, autopilot mode) set distance to the tracking point used on the leg
                                    # in front of the vehicle for guidance
        "radius_in": DE, # (station mode)
        "radius_out": DE, # (station mode)
    },
}
