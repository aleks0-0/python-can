# -*- coding: utf-8 -*-
"""
Defines CAN constants.
"""
import ctypes

canMSG_EXT = 0x0004

CAN_RAW =       1
CAN_BCM =       2
MSK_ARBID =     0x1FFFFFFF
MSK_FLAGS =     0xE0000000

PF_CAN  =       29
SOCK_RAW =      3
SOCK_DGRAM =    2
AF_CAN =        PF_CAN

SIOCGIFINDEX =  0x8933 
SIOCGSTAMP =    0x8906
EXTFLG =        0x0004

SKT_ERRFLG  =   0x0001
SKT_RTRFLG  =   0x0002

PYCAN_ERRFLG =  0x0020
PYCAN_STDFLG =  0x0002
PYCAN_RTRFLG =  0x0001

ID_TYPE_EXTENDED = True
ID_TYPE_STANDARD = False

ID_TYPE_29_BIT = ID_TYPE_EXTENDED
ID_TYPE_11_BIT = ID_TYPE_STANDARD

REMOTE_FRAME = True
DATA_FRAME = False
WAKEUP_MSG = True
ERROR_FRAME = True

DRIVER_MODE_SILENT = False
DRIVER_MODE_NORMAL = (not DRIVER_MODE_SILENT)

STD_ACCEPTANCE_MASK_ALL_BITS = (2**11 - 1)
MAX_11_BIT_ID = STD_ACCEPTANCE_MASK_ALL_BITS

EXT_ACCEPTANCE_MASK_ALL_BITS = (2**29 - 1)
MAX_29_BIT_ID = EXT_ACCEPTANCE_MASK_ALL_BITS

MAX_DEVICE_DESCR_LENGTH = 256
MAX_MANUFACTURER_NAME_LENGTH = 256
MAX_FW_VERSION_LENGTH = 8
FW_VERSION_ARRAY = ctypes.c_ubyte * MAX_FW_VERSION_LENGTH
MAX_HW_VERSION_LENGTH = 8
HW_VERSION_ARRAY = ctypes.c_ubyte * MAX_HW_VERSION_LENGTH
MAX_CARD_SN_LENGTH = 8
CARD_SN_ARRAY = ctypes.c_ubyte * MAX_CARD_SN_LENGTH
MAX_TRANS_SN_LENGTH = 8
TRANS_SN_ARRAY = ctypes.c_ubyte * MAX_TRANS_SN_LENGTH