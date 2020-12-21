"""
Enable basic CAN over a ECOM USB device.
"""

import logging
import time

from typing import Optional
from can import CanError, Message, BusABC
from can.bus import BusState
from can.util import len2dlc, dlc2len
from .basic import *
from .ecom import Ecom, Ecommlib

# Set up logging
log = logging.getLogger("can.pcan")

class EcomBus(BusABC):
    def __init__(
        self,
        channel="0",
        state=BusState.ACTIVE,
        bitrate=250000,
        *args,
        **kwargs
    ):
        """An ECOM USB interface to CAN.

        On top of the usual :class:`~can.Bus` methods provided,
        the PCAN interface includes the :meth:`~can.interface.pcan.PcanBus.flash`
        and :meth:`~can.interface.pcan.PcanBus.status` methods.

        :param str channel:
            The can interface name. An example would be '512000'
            Default is '0' - This takes the first available interface.

        :param int bitrate:
            Bitrate of channel in bit/s.
            Default is 250 kbit/s.
            Ignored if using CanFD.

        """
        self.channel_info = channel
        self.fd = kwargs.get("fd", False)

        if bitrate==500000:
            baud = '500k'
        else:
            baud='250K'

        self.ecomlib = Ecommlib()
        if isinstance(channel, str):
            # if it is a string assume it is formatted as info.
            channel = channel.strip('ECOM: ')

        self.device = self.ecomlib.open_device(int(channel),baud)

        # if state is BusState.ACTIVE or state is BusState.PASSIVE:
        #     self.state = state
        # else:
        #     raise ArgumentError("BusState must be Active or Passive")

        # if result != PCAN_ERROR_OK:
        #     raise PcanError(self._get_formatted_error(result))

        # if HAS_EVENTS:
        #     self._recv_event = CreateEvent(None, 0, 0, None)
        #     result = self.m_objPCANBasic.SetValue(
        #         self.m_PcanHandle, PCAN_RECEIVE_EVENT, self._recv_event
        #     )
        #     if result != PCAN_ERROR_OK:
        #         raise PcanError(self._get_formatted_error(result))

        super().__init__(channel=channel, state=state, bitrate=bitrate, *args, **kwargs)

#     def _get_formatted_error(self, error):
#         """
#         Gets the text using the GetErrorText API function.
#         If the function call succeeds, the translated error is returned. If it fails,
#         a text describing the current error is returned. Multiple errors may
#         be present in which case their individual messages are included in the
#         return string, one line per error.
#         """

#         def bits(n):
#             """
#             Iterate over all the set bits in `n`, returning the masked bits at
#             the set indices
#             """
#             while n:
#                 # Create a mask to mask the lowest set bit in n
#                 mask = ~n + 1
#                 masked_value = n & mask
#                 yield masked_value
#                 # Toggle the lowest set bit
#                 n ^= masked_value

#         stsReturn = self.m_objPCANBasic.GetErrorText(error, 0)
#         if stsReturn[0] != PCAN_ERROR_OK:
#             strings = []

#             for b in bits(error):
#                 stsReturn = self.m_objPCANBasic.GetErrorText(b, 0)
#                 if stsReturn[0] != PCAN_ERROR_OK:
#                     text = "An error occurred. Error-code's text ({0:X}h) couldn't be retrieved".format(
#                         error
#                     )
#                 else:
#                     text = stsReturn[1].decode("utf-8", errors="replace")

#                 strings.append(text)

#             complete_text = "\n".join(strings)
#         else:
#             complete_text = stsReturn[1].decode("utf-8", errors="replace")

#         return complete_text

#     def status(self):
#         """
#         Query the PCAN bus status.

#         :rtype: int
#         :return: The status code. See values in **basic.PCAN_ERROR_**
#         """
#         return self.m_objPCANBasic.GetStatus(self.m_PcanHandle)

#     def status_is_ok(self):
#         """
#         Convenience method to check that the bus status is OK
#         """
#         status = self.status()
#         return status == PCAN_ERROR_OK

#     def reset(self):
#         """
#         Command the PCAN driver to reset the bus after an error.
#         """
#         status = self.m_objPCANBasic.Reset(self.m_PcanHandle)
#         return status == PCAN_ERROR_OK

    def _recv_internal(self, timeout):

        # if HAS_EVENTS:
        #     # We will utilize events for the timeout handling
        #     timeout_ms = int(timeout * 1000) if timeout is not None else INFINITE
        # elif timeout is not None:
        #     # Calculate max time
        #     end_time = time.perf_counter() + timeout

        # log.debug("Trying to read a msg")

        result = None
        is_extended_id = False

        if self.fd:
            pass
            #fd not currently supported.
        else:
            # Look for extended ID messages.
            msg = self.device.receive_ext()
            if(msg):
                is_extended_id = True
            else:
                # Look for 11 bit messages.
                msg = self.device.receive()
                if (msg):
                    is_extended_id = False
                else:
                    # no message was received.
                    return None, True

        dlc = msg.data_length
        timestamp = 0.0
        # TODO Implement time stamp
        # boottimeEpoch + (
        #     (
        #         itsTimeStamp.micros
        #         + 1000 * itsTimeStamp.millis
        #         + 0x100000000 * 1000 * itsTimeStamp.millis_overflow
        #     )
        #     / (1000.0 * 1000.0)
        # )

        rx_msg = Message(
            timestamp=0.0, # Add timestamp later.
            arbitration_id = msg.msg_id,
            is_extended_id = is_extended_id,
            is_remote_frame=False,
            is_error_frame=False,
            dlc=dlc,
            data=msg.data,
            is_fd=False,
            bitrate_switch=False,
            error_state_indicator=False,
        )

        return rx_msg, True

    def send(self, msg, timeout=None):
        if msg.is_extended_id:
            self.device.transmit_ext(msg)
        else:
            self.device.transmit(msg)
        # msgType = (
        #     PCAN_MESSAGE_EXTENDED.value
        #     if msg.is_extended_id
        #     else PCAN_MESSAGE_STANDARD.value
        # )
        # if msg.is_remote_frame:
        #     msgType |= PCAN_MESSAGE_RTR.value
        # if msg.is_error_frame:
        #     msgType |= PCAN_MESSAGE_ERRFRAME.value
        # if msg.is_fd:
        #     raise ValueError("is_fd=True, FD not supported by Ecom interface.")
        #     msgType |= PCAN_MESSAGE_FD.value
        # if msg.bitrate_switch:
        #     msgType |= PCAN_MESSAGE_BRS.value
        # if msg.error_state_indicator:
        #     msgType |= PCAN_MESSAGE_ESI.value

        # # create a TPCANMsg message structure
        # if platform.system() == "Darwin":
        #     CANMsg = TPCANMsgMac()
        # else:
        #     CANMsg = TPCANMsg()

        # # configure the message. ID, Length of data, message type and data
        # CANMsg.ID = msg.arbitration_id
        # CANMsg.LEN = msg.dlc
        # CANMsg.MSGTYPE = msgType

        # # if a remote frame will be sent, data bytes are not important.
        # if not msg.is_remote_frame:
        #     # copy data
        #     for i in range(CANMsg.LEN):
        #         CANMsg.DATA[i] = msg.data[i]

        # log.debug("Data: %s", msg.data)
        # log.debug("Type: %s", type(msg.data))

        # result = self.m_objPCANBasic.Write(self.m_PcanHandle, CANMsg)

        # if result != PCAN_ERROR_OK:
        #     raise PcanError("Failed to send: " + self._get_formatted_error(result))

#     def flash(self, flash):
#         """
#         Turn on or off flashing of the device's LED for physical
#         identification purposes.
#         """
#         self.m_objPCANBasic.SetValue(
#             self.m_PcanHandle, PCAN_CHANNEL_IDENTIFYING, bool(flash)
#         )

#     def shutdown(self):
#         super().shutdown()
#         self.m_objPCANBasic.Uninitialize(self.m_PcanHandle)

#     @property
#     def state(self):
#         return self._state

#     @state.setter
#     def state(self, new_state):
#         # declare here, which is called by __init__()
#         self._state = new_state  # pylint: disable=attribute-defined-outside-init

#         if new_state is BusState.ACTIVE:
#             self.m_objPCANBasic.SetValue(
#                 self.m_PcanHandle, PCAN_LISTEN_ONLY, PCAN_PARAMETER_OFF
#             )

#         elif new_state is BusState.PASSIVE:
#             # When this mode is set, the CAN controller does not take part on active events (eg. transmit CAN messages)
#             # but stays in a passive mode (CAN monitor), in which it can analyse the traffic on the CAN bus used by a
#             # PCAN channel. See also the Philips Data Sheet "SJA1000 Stand-alone CAN controller".
#             self.m_objPCANBasic.SetValue(
#                 self.m_PcanHandle, PCAN_LISTEN_ONLY, PCAN_PARAMETER_ON
#             )

    @staticmethod
    def _detect_available_configs():
        channels = []

        ecom_lib = Ecommlib()

        dev_list = ecom_lib.find_all_devices()

        if(0 == len(dev_list)):
            print('No ECOM Devices found.')
        else:
            for dev in dev_list:
                channels.append({"interface": "ecom", "channel": str(dev), "supports_fd": False, "info":f"ECOM:{dev}"})

        return channels
#         interfaces = []
#         for i in range(16):
#             interfaces.append(
#                 {
#                     "id": TPCANHandle(PCAN_PCIBUS1.value + i),
#                     "name": "PCAN_PCIBUS" + str(i + 1),
#                 }
#             )
#         for i in range(16):
#             interfaces.append(
#                 {
#                     "id": TPCANHandle(PCAN_USBBUS1.value + i),
#                     "name": "PCAN_USBBUS" + str(i + 1),
#                 }
#             )
#         for i in range(2):
#             interfaces.append(
#                 {
#                     "id": TPCANHandle(PCAN_PCCBUS1.value + i),
#                     "name": "PCAN_PCCBUS" + str(i + 1),
#                 }
#             )
#         for i in range(16):
#             interfaces.append(
#                 {
#                     "id": TPCANHandle(PCAN_LANBUS1.value + i),
#                     "name": "PCAN_LANBUS" + str(i + 1),
#                 }
#             )
#         for i in interfaces:
#             error, value = library_handle.GetValue(i["id"], PCAN_CHANNEL_CONDITION)
#             if error != PCAN_ERROR_OK or value != PCAN_CHANNEL_AVAILABLE:
#                 continue
#             has_fd = False
#             error, value = library_handle.GetValue(i["id"], PCAN_CHANNEL_FEATURES)
#             if error == PCAN_ERROR_OK:
#                 has_fd = bool(value & FEATURE_FD_CAPABLE)
#             channels.append(
#                 {"interface": "pcan", "channel": i["name"], "supports_fd": has_fd}
#             )
#         return channels

#     def status_string(self) -> Optional[str]:
#         """
#         Query the PCAN bus status.
#         :return: The status in string.
#         """
#         if self.status() in PCAN_DICT_STATUS:
#             return PCAN_DICT_STATUS[self.status()]
#         else:
#             return None


# class PcanError(CanError):
#     """
#     A generic error on a PCAN bus.
#     """
