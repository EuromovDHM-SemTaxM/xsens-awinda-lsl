import logging

from qtm.packet import QRTPacketType
from qtm.packet import QRTPacket, QRTEvent
from qtm.packet import RTheader, RTEvent, RTCommand

from xsenslsl.packet import XSensHeader, XSensPacketType

logger = logging.getLogger("xsenslsl.receiver")


class Receiver(object):
    def __init__(self, handlers):
        self._handlers = handlers
        self._received_data = b""

    def datagram_received(self, data):
        """ Received from XSense, to be routed accordingly """
        self._received_data += data
        h_size = XSensHeader.size
        data = self._received_data
        data_len = len(data)

        while data_len >= h_size:
            size, type_ = XSensHeader.unpack_from(data, 0)
            if data_len >= size:
                self._parse_received(data[h_size:size], type_)
                data = data[size:]
                data_len = len(data)
            else:
                break

        self._received_data = data

    def _parse_received(self, data, type_):
        type_ = XSensPacketType(type_)
        handler = (
            self._handlers[type_](data)
            if type_ in self._handlers
            else self._handlers["default"]
        )
        try:
            handler(data)
        except KeyError:
            logger.error("Non handled packet type! - %s", type_)
