# file can_capture_interface.py

import sys
sys.path.insert (0, '../')

import ctypes
import inspect
import os
import time
from ctypes import c_char, c_ulong, c_ushort, c_double

from cmw_can import can_message as Can_Message
import platform


#%% ECommlib Class
class Ecommlib:

    def __init__(self,**kwargs):

        if(platform.architecture()[0] != '32bit'):
            #print("Error: The Ecom DLL requires you to run a 32 bit version on python.")
            raise OSError ("Error: The Ecom DLL requires you to run a 32 bit version of python.")

        (path,f_name) = os.path.split(inspect.getfile(inspect.currentframe()))
        self._library = ctypes.windll.LoadLibrary(path + "/ecommlib.dll")

        self._dev_search_handle = None

    #Error Return Values
    #Note that if CANTransmit() or CANTransmitEx() returns
    #an error code other than one defined below that it
    #is returning the Error Code Capture Register from the SJA1000
    #CAN controller.  These error codes are explained in
    #Section 5.2.3 of the SJA1000 Application Note AN97076
    #and Section 6.4.9 of SJA1000 Data-Sheet
    CAN_Transmit_Error = {
    0xFF : 'ERR_COULD_NOT_START_CAN',          #failed to send commend to start CAN controller
    0xFE : 'ERR_COULD_NOT_ENUMERATE',          #switching firmware and/or enumeration on USB bus failed
    0xFD : 'ERR_SERIAL_NUMBER_NOT_FOUND',      #device with passed serial number not found
    0xFC : 'ERR_DEVICE_CLOSED',                #the device at the received handle is closed
    0xFB : 'ERR_NO_DEVICES_ATTACHED',          #No devices found (wait/unplug and try again)
    0xFA : 'ERR_INVALID_FIRMWARE',             #multiple causes - possibly a bad DeviceHandle
    0xF9 : 'ERR_ALREADY_OPEN_AS_CAN',          #device open already (existing device handle returned)
    0xF8 : 'ERR_ALREADY_OPEN_AS_SERIAL',       #device open already (existing device handle returned)
    0xF7 : 'ERR_NO_FREE_DEVICE',               #all attached devices are already open
    0xF6 : 'ERR_INVALID_HANDLE',               #invalid device handle passed
    0xF5 : 'ERR_CAN_COULD_NOT_READ_STATUS',    #Could not retrieve status from CAN device
    0xF4 : 'ERR_USB_TX_FAILED',                #A failured occurred transfering on the USB bus to the device
    0xF3 : 'ERR_USB_RX_FAILED',                #A failured occurred transfering on the USB bus to the device
    0xF2 : 'ERR_USB_TX_LENGTH_MISMATCH',       #Unexpected error transfering on USB bus
    0xF1 : 'ERR_CAN_TX_TIMEOUT',               #tx timeout occurred (msg may send on bus)
    0xF0 : 'ERR_CAN_TX_ABORTED',               #synch. transfer aborted due to timeout
    0xEF : 'ERR_CAN_TX_ABORTED_UNEXPECTED',    #synch. transfer unexpectedly aborted
    0xEE : 'ERR_NULL_DEVICE_HANDLE',           #You passed a NULL device handle
    0xED : 'ERR_INVALID_DEVICE_HANDLE',
    0xEC : 'ERR_CAN_TX_BUFFER_FULL',           #The async transfer buffer is full, wait and try again
    0xEB : 'ERR_CAN_RX_ZEROLENGTH_READ',       #Reading the CAN bus returned a zero length msg (unexpected)
    0xEA : 'ERR_CAN_NOT_OPENED',               #Device has not been opened as CAN
    0xE9 : 'ERR_SERIAL_NOT_OPENED',            #Device has not been opened as Serial
    0xE8 : 'ERR_COULD_NOT_START_THREAD',       #Thread could not be started
    0xE7 : 'ERR_THREAD_STOP_TIMED_OUT',        #Thread did not stop in a reasonable amount of time
    0xE6 : 'ERR_THREAD_ALREADY_RUNNING',
    0xE5 : 'ERR_RXTHREAD_ALREADY_RUNNING',     #The receive MessageHandler thread is already running
    0xE4 : 'ERR_CAN_INVALID_SETUP_PROPERTY',   #An invalid property was received by the CANSetupDevice() function
    0xE3 : 'ERR_CAN_INVALID_SETUP_COMMAND',    #An invalid flag was received by the CANSetupDevice() function
    0xE2 : 'ERR_COMMAND_FAILED',               #The command passed to SetupDevice failed
    0xE1 : 'ERR_SERIAL_INVALID_BAUD',
    0xE0 : 'ERR_DEVICE_UNPLUGGED',             #The device was physically removed from the CAN bus after being attached
    0xDF : 'ERR_ALREADY_OPEN',                 #The device is already open
    0xDE : 'ERR_NULL_DRIVER_HANDLE',           #Could not retrieve a handle to the USB driver
    0xDD : 'ERR_SER_TX_BUFFER_FULL',
    0xDC : 'ERR_NULL_DEV_SEARCH_HANDLE',       #A null device search handle was passed
    0xDB : 'ERR_INVALID_DEV_SEARCH_HANDLE',    #An invalid search handle was passed
    #0xD9 : 'ERR_CONFIG_COMMAND_TIME_OUT',
    0xD8 : 'ERR_NO_LONGER_SUPPORTED',          #This feature has been removed and is only supported for legacy purposes
    0xD7 : 'ERR_NULL_PTR_PASSED',
    0xD6 : 'ERR_INVALID_INPUT_BUFFER',         #Unexpected error, driver received invalid input buffer to IOCTL command
    0xD5 : 'ERR_INVALID_INPUT_COMMAND',        #Unexpected error, driver received invalid input buffer to IOCTL command
    0xD4 : 'ERR_ALREADY_OPEN_DIFFERENT_BAUD',  #CAN device is already opened by another application but at a different baud rate
    0xD3 : 'ERR_ALREADY_OPEN_BY_PROCESS',      #Calling process already has this device open, or the unique ID is already in use
    0xD2 : 'ERR_TOO_MANY_CONNECTS',            #Too many applications have connected to the driver (limit to 16)

    #These are some error codes returned by CANTransmit() and CANTransmitEx()
    0xD9 : 'ERR_CAN_TX_NO_ACK',               #device is probably alone on bus
    0x14 : 'ERR_SJA1000_EXIT_RESET',
    0x15 : 'ERR_SJA1000_ENTER_RESET',
    }

    #Pass this instead of a serial number to CANOpen() or CANOpenFiltered() to find
    #the first CAN device attached to the USB bus that is not in use.
    #You can then retrieve the serial number by passing the returned handle to GetDeviceInfo()
    CAN_FIND_NEXT_FREE =             0x00

    control_byte_errors = {
    0x11 : 'CAN_ERR_BUS',                     #A CAN Bus error has occurred (DataByte contains ErrorCaptureCode Register)
    0x12 : 'CAN_ERR_BUS_OFF_EVENT',           #Bus off due to error
    0x13 : 'CAN_ERR_RESET_AFTER_BUS_OFF',     #Error reseting SJA1000 after bus off event
    0x16 : 'CAN_ERR_RX_LIMIT_REACHED',        #The default rx error limit (96) has been reached
    0x17 : 'CAN_ERR_TX_LIMIT_REACHED',        #The default tx error limit (96) has been reached
    0x18 : 'CAN_BUS_BACK_ON_EVENT',           #Bus has come back on after a bus off event due to errors
    0x19 : 'CAN_ARBITRATION_LOST',            #Arbitration lost (DataByte contains location lost) see SJA1000 datasheet
    0x1A : 'CAN_ERR_PASSIVE',                 #SJA1000 has entered error passive mode
    0x1B : 'CAN_ERR_OVERRUN',                 #Embedded firmware has received a receive overrun
    0x1C : 'CAN_ERR_OVERRUN_PC',              #PC driver received a receive overrun
    0x20 : 'ERR_ERROR_FIFO_OVERRUN',          #Error buffer full - new errors will be lost
    0x21 : 'ERR_EFF_RX_FIFO_OVERRUN',         #EFF Receive buffer full - messages will be lost
    0x22 : 'ERR_SFF_RX_FIFO_OVERRUN',         #SFF Receive buffer full - messages will be lost

    0x23 : 'CAN_RECEIVED_ERROR_MESSAGES',     #Received error messages
    0x24 : 'CAN_RECEIVED_SFF_MESSAGES',       #Received error messages
    0x25 : 'CAN_RECEIVED_EFF_MESSAGES',       #Received error messages
    }




    #Status Return Values
    #The following return codes signify the error free
    # completion of a function
    error_free_return_values = {
    0x00 : 'ECI_NO_ERROR',
    0x88 : 'CAN_NO_RX_MESSAGES',
    0x89 : 'CAN_NO_ERROR_MESSAGES',
    0x80 : 'ECI_NO_MORE_DEVICES',
    }

    #Setup Commands and valid properties for each used by CANSetupDevice()
    setup_command ={
    0x00 : 'CAN_CMD_TRANSMIT',
    0x10 : 'CAN_CMD_TIMESTAMPS',

        }

    setup_command_property = {
    0x00 : 'CAN_PROPERTY_ASYNC',
    0x01 : 'CAN_PROPERTY_SYNC',
    0x10 : 'CAN_PROPERTY_RECEIVE_TS',
    0x11 : 'CAN_PROPERTY_DONT_RECEIVE_TS',
        }

    #Setup Properties for CANSetupDevice()

    #The following constants are flags that are passed in the second parameter
    #of the ReceiveCallback function
    receive_callback_flags = {
    0x30 : 'CAN_EFF_MESSAGES',            #context byte is number of messages in EFF buffer
    0x31 : 'CAN_SFF_MESSAGES',            #context byte is number of messages in SFF buffer
    0x32 : 'CAN_ERR_MESSAGES',            #context byte is number of messages in error buffer
    0x33 : 'SER_BYTES_RECEIVED',          #context byte is number of messages in Serial receive buffer
    }

    #The following flags are passed to CANQueueSize to set which queue to check the size of
    #for a device open as CAN
    can_queue_size_flags = {
    0  : 'CAN_GET_EFF_SIZE',            #Retrieve the current number of messages waiting to be received
    1  : 'CAN_GET_MAX_EFF_SIZE',        #Get the max size of the EFF buffer  (fixed)
    2  : 'CAN_GET_SFF_SIZE',            #...
    3  : 'CAN_GET_MAX_SFF_SIZE',        #...  (fixed)
    4  : 'CAN_GET_ERROR_SIZE',          #...
    5  : 'CAN_GET_MAX_ERROR_SIZE',      #...  (fixed)
    6  : 'CAN_GET_TX_SIZE',             #...
    7  : 'CAN_GET_MAX_TX_SIZE',         #...  (fixed)
    #for a device open as serial
    8  : 'SER_GET_RX_SIZE',             #...
    9  : 'SER_GET_MAX_RX_SIZE',         #...  (fixed)
    10 : 'SER_GET_TX_SIZE',             #...
    11 : 'SER_GET_MAX_TX_SIZE',         #...  (fixed)
    }


    #The following constants are flags that
    #can be passed to the StartDeviceSearch function
    start_device_search_flags = {
    'FIND_OPEN'   : 0x82,
    'FIND_UNOPEN' : 0x83,
    'FIND_ALL'    : 0x87,
    }

    #The following are the defined baud rates for CAN
    can_baud_rate = {
    '250K' : 0x00,
    '500K' : 0x01,
    '1MB'  : 0x02,
    '125K' : 0x03,
    }

    #OR this with the baud rate to enable listen-only mode
    CAN_LISTEN_ONLY =  0x80

    # Extended Can ID Message
    #
    class EFFMessage(ctypes.Structure):
        _fields_ = [
        ('ID'     , c_ulong),
        ('data1'  , c_char ),
        ('data2'  , c_char ),
        ('data3'  , c_char ),
        ('data4'  , c_char ),
        ('data5'  , c_char ),
        ('data6'  , c_char ),
        ('data7'  , c_char ),
        ('data8'  , c_char ),
        ('options', c_char ),          #BIT 6 = remote frame bit
                  #set BIT 4 on transmissions for self reception
        ('data_length', c_char) ,
        ('time_stamp',  c_ulong)   #Extending timestamp to support 4 byte TS mode... shouldnt hurt anything for older code using 2 byte mode
        ]

    # Standard CAN ID message
    class SFFMessage (ctypes.Structure):
        _fields_ = [
        ('IDH'    , c_char ),
        ('IDL'    , c_char ),
        ('data1'  , c_char ),
        ('data2'  , c_char ),
        ('data3'  , c_char ),
        ('data4'  , c_char ),
        ('data5'  , c_char ),
        ('data6'  , c_char ),
        ('data7'  , c_char ),
        ('data8'  , c_char ),
        ('options', c_char ),  #BIT 6 = remote frame bit
                          #set BIT 4 on transmissions for self reception
        ('data_length', c_char) ,
        ('time_stamp',  c_ulong)  #Extending timestamp to support 4 byte TS mode... shouldnt hurt anything for older code using 2 byte mode
    ]

    class DeviceInfo (ctypes.Structure):
        _fields_ = [
        ('SerialNumber', c_ulong),   #Device serial number
        ('CANOpen'     , c_char ),   #is device opened as CAN
        ('SEROpen'     , c_char ),   #is device opened as Serial
        ('_reserved'   , c_char ),   #legacy support - was used to indicate if message handler was running - now its always running
        ('SyncCANTx'   , c_char ),   #always FALSE if returned by FindNextDevice
        ('DeviceHandle', c_ulong),   #NULL if structure returned by FindNextDevice - required b/c search is across all processes using the DLL and
                                #HANDLE will be invalid across multiple processes.  Each process must keep track of their open HANDLEs

        ('reserved1',    c_ulong),
        ('reserved2',    c_ulong),
        ('reserved3',   c_ushort),
        ]

        def to_string(self):
            result =""

            result += str(self.SerialNumber)
            return result

    class ErrorMessage (ctypes.Structure):
        _fields_ = [
        ('ErrorFIFOSize' , c_ulong  ),
        ('ErrorCode'     , c_char   ),
        ('ErrorData'     , c_char   ),
        ('Timestamp'     , c_double ),
        ('reserved1'     , c_char   ),
        ('reserved2'     , c_char   ),
    ]

    #steve added for message event handling
    #public delegate void pMessageHandler(UInt32 DeviceHandle, byte Flag, uint FlagInfo, ref byte[] b)
    #public delegate byte pMessageHandler(UInt32 DeviceHandle, byte Flag, uint FlagInfo, IntPtr UserData)


    #public static extern Byte SetCallbackFunction(UInt32 DeviceHandle, pMessageHandler ReceiveCallback, IntPtr UserData)


    #[DllImport("ecommlib.dll")]
    #public static extern UInt32 CANOpen(UInt32 Serial, Byte baud, ref Byte error)
    def open_device(self,serial_number, baud='250K'):
        error_val = c_char(0)
        handle = self._library.CANOpen(c_ulong(serial_number), c_char(Ecommlib.can_baud_rate[baud]), ctypes.byref(error_val))
        #print("ECOM Handle: " + str(handle))
        return Ecom(handle = handle, serial_number = serial_number, ecom_lib = self)

    #[DllImport("ecommlib.dll")]
    #public static extern Byte CANTransmitMessageEx(UInt32 DeviceHandle, ref EFFMessage message)
    def transmit_extended_id(self,handle,msg):
        # Create an 8 byte array and copy the data into it.
        data = bytearray(8)
        for idx, val in enumerate(msg.data):
            data[idx] = val

        message = Ecommlib.EFFMessage(ID = msg.arbitration_id,
                              data1 = data[0],
                              data2 = data[1],
                              data3 = data[2],
                              data4 = data[3],
                              data5 = data[4],
                              data6 = data[5],
                              data7 = data[6],
                              data8 = data[7],
                              data_length = msg.dlc)
        self._library.CANTransmitMessageEx(handle,ctypes.byref(message))


    def transmit(self,handle,msg):
        '''Transmit an 11-bit id message.'''
        # Create an 8 byte array and copy the data into it.
        data = bytearray(8)
        for idx, val in enumerate(msg.data):
            data[idx] = val

        message = Ecommlib.SFFMessage(IDH = ( msg.arbitration_id & 0x0000FF00) >> 8,
                                      IDL = ( msg.arbitration_id & 0x000000FF),
                                      data1 = data[0],
                                      data2 = data[1],
                                      data3 = data[2],
                                      data4 = data[3],
                                      data5 = data[4],
                                      data6 = data[5],
                                      data7 = data[6],
                                      data8 = data[7],
                                      data_length = msg.dlc)

        self._library.CANTransmitMessage(handle,ctypes.byref(message))

    #[DllImport("ecommlib.dll")]
    #public static extern Byte CANReceiveMessageEx(UInt32 DeviceHandle, ref EFFMessage message)
    def receive_extended_id(self, handle):
        message = Ecommlib.EFFMessage()
        result = self._library.CANReceiveMessageEx(handle,ctypes.byref(message))
        translated_result = Ecommlib.error_free_return_values.get(result)
        if (translated_result == 'CAN_NO_RX_MESSAGES'):
            return None
        else:
            data_length = message.data_length[0]
            rx_data = bytearray([message.data1[0], message.data2[0], message.data3[0], message.data4[0], message.data5[0], message.data6[0], message.data7[0], message.data8[0]])
            rx_data = rx_data[0:data_length]
            #print('Msg Rxed : ID ' + hex((message.IDH[0] << 8) + message.IDL[0]) +' Data: '+ str(rx_data))
            return Can_Message.Can_Message(message.ID,
                               rx_data)
    #[DllImport("ecommlib.dll")]
    #public static extern Byte CANReceiveMessage(UInt32 DeviceHandle, ref SFFMessage message)
    def receive(self, handle):
        message = Ecommlib.SFFMessage()
        result = self._library.CANReceiveMessage(handle,ctypes.byref(message))
        translated_result = Ecommlib.error_free_return_values.get(result)
        if (translated_result == 'CAN_NO_RX_MESSAGES'):
            return None
        else:
            # print('IDH: '+ str(message.IDH[0]) +' ' + str(type(message.IDH[0])))
            # print('IDL: '+ str(message.IDL[0]) +' ' + str(type(message.IDL)))
            data_length = message.data_length[0]
            rx_data = bytearray([message.data1[0], message.data2[0], message.data3[0], message.data4[0], message.data5[0], message.data6[0], message.data7[0], message.data8[0]])
            rx_data = rx_data[0:data_length]
            #print('Msg Rxed : ID ' + hex((message.IDH[0] << 8) + message.IDL[0]) +' Data: '+ str(rx_data))
            return Can_Message.Can_Message((message.IDH[0] << 8) + message.IDL[0],
                               rx_data)


    #[DllImport("ecommlib.dll")]
    #public static extern Byte GetErrorMessage(UInt32 DeviceHandle, ref ErrorMessage message)
    #[DllImport("ecommlib.dll")]
    #public static extern Byte GetDeviceInfo(UInt32 DeviceHandle, ref DeviceInfo deviceInfo)
    #[DllImport("ecommlib.dll")]
    #public static extern Byte CloseDevice(UInt32 DeviceHandle)
    #[DllImport("ecommlib.dll")]
    #public static extern UInt32 CANOpenFiltered(UInt32 SerialNumber, Byte baud, UInt32 code, UInt32 mask, ref Byte error)
    #[DllImport("ecommlib.dll")]
    #public static extern Byte CANSetupDevice(UInt32 DeviceHandle, Byte SetupCommand, Byte SetupProperty)
    def set_async_tx(self, handle):
        result = self._library.CANSetupDevice(handle,0, 0)
    #[DllImport("ecommlib.dll")]
    #public static extern UInt32 SerialOpen(UInt16 SerialNumber, Byte baud, ref Byte error)
    #[DllImport("ecommlib.dll")]
    #public static extern Byte SerialWrite(UInt32 DeviceHandle, ref Byte buffer, ref Int32 length)
    #[DllImport("ecommlib.dll")]
    #public static extern Byte SerialRead(UInt32 DeviceHandle, ref Byte buffer, ref Int32 buffer_length)
    #[DllImport("ecommlib.dll")]
    #public static extern Int32 GetQueueSize(UInt32 DeviceHandle, Byte flag)
    #[DllImport("ecommlib.dll")]
    #public static extern void GetFriendlyErrorMessage(Byte error, StringBuilder buffer, Int32 buffer_size)

    #Use the following functions to enumerate through devices
    #[DllImport("ecommlib.dll")]
    #public static extern UInt32 StartDeviceSearch(Byte flag)

    def _start_device_search(self):
        self._dev_search_handle = self._library.StartDeviceSearch(c_ulong(0))

    #[DllImport("ecommlib.dll")]
    #public static extern Byte CloseDeviceSearch(UInt32 searchHandle)
    def _close_device_search(self):
        self._dev_search_handle = self._library.CloseDeviceSearch(c_ulong(self._dev_search_handle))


    #[DllImport("ecommlib.dll")]
    #public static extern Byte FindNextDevice(UInt32 searchHandle, ref DeviceInfo deviceInfo)
    def _find_next_device(self):
        dev_info = Ecommlib().DeviceInfo()
        return_value = self._library.FindNextDevice(c_ulong(self._dev_search_handle), ctypes.byref(dev_info))
        #print(dev_info.to_string())
        if(Ecommlib.error_free_return_values.get(return_value) == 'ECI_NO_ERROR'):
            return dev_info.SerialNumber
        else:
            return None


    def _list_all_devices(self):
        device_list = []
        while True:
            # ASW added a sleep here we were getting occasional hiccups in reading the devices.
            time.sleep(0.05)
            dev = self._find_next_device()
            if dev is None:
                break;
            else:
                device_list.append(dev)

        return device_list

    def find_all_devices(self):
        """Finds all ecom devices installed on the system and returns them as a list.
        This function will retry the find operation if an invalid ecom serial number is found.
        This is due to a known bug in the ecom dll."""

        dev_list = list()

        retry = 0

        while True:

            self._start_device_search()

            dev_list = self._list_all_devices()

            self._close_device_search()

            if len(dev_list) == 0:
                break

            bad = False
            for dev in dev_list:
                if int(dev) < 10000:
                    # An invalid serial number was found.
                    bad = True

            if(bad):
                retry += 1
                if retry > 5:
                    break
                else:
                    # there are some invalid serial numbers in there.
                    print("Retrying find ECOMS")
            else:
                break

        return dev_list



#%% Ecom Class
class Ecom:
    '''
    This class represents a single Ecom Device
    This class follows a standard CAN interface definition that can allow any
    CAN interface to be used.

    :kwarg serial_number - an integer nu
    '''
    def __init__(self, **kwargs):

        self._serial_number = kwargs.get("serial_number")

        self._handle = kwargs.get("handle")

        self._ecom_lib = kwargs.get("ecom_lib")

        self._ecom_lib.set_async_tx(self._handle)

    @property
    def handle(self):
        return self._handle

    @handle.setter
    def handle(self, handle):
        self._handle = handle

    @property
    def serial_number(self):
        return self._serial_number

    def receive(self):
        '''Check the ecom interface for an 11 bit CAN message'''
        return self._ecom_lib.receive(self.handle)

    def receive_ext(self):
        '''Check the ecom interface for an extended id (29 bit) message'''
        return self._ecom_lib.receive_extended_id(self.handle)

    def flush_receive(self):
        '''Recieve all back logged messages and throw them away.'''
        while(self.receive() is not None):
            pass

    def transmit(self, message):
        '''transmit a message on the ecom interface'''
        return self._ecom_lib.transmit(self.handle, message)

    def transmit_ext(self, message):
        '''transmit an extended id (29 bit) message'''
        return self._ecom_lib.transmit_extended_id(self.handle,message)