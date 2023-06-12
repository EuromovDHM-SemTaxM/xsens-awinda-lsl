import asyncio
import collections
import contextlib
import logging
from xsenslsl.packet import *
from xsenslsl.receiver import Receiver

logger = logging.getLogger(__name__)


class XSensException(Exception):
    """
        Basic XSense Exception
    """

    def __init__(self, value):
        super(XSensException, self).__init__()
        self.value = value

    def __str__(self):
        return repr(self.value)


class XSensProtocol(asyncio.DatagramProtocol):
    """
        XSens Network Streamer Protocol implementation
        Should be constructed by ::xsens.connect
    """

    def __init__(self, *, loop=None, on_disconnect=None, on_event=None):
        self._received_data = b""

        self.on_disconnect = on_disconnect
        self.on_packet = None

        self.request_queue = collections.deque()
        self.event_future = None
        self._start_streaming = False

        self.loop = loop or asyncio.get_event_loop()
        self.transport = None
        self._handlers = {
            "default": self._on_data,
        }
        self._receiver = Receiver(self._handlers)

    def connection_made(self, transport):
        logger.info("Connected")
        self.transport = transport

    def set_on_frame(self, on_packet):
        """ Set callback to use when packet arrives """
        self.on_packet = on_packet
        self._start_streaming = on_packet is not None

    def datagram_received(self, data, addr):
        """ Received from QTM and route accordingly """
        self._receiver.datagram_received(data)

    def _deliver_promise(self, data):
        with contextlib.suppress(IndexError):
            future = self.request_queue.pop()
            future.set_result(data)

    def _on_data(self, packet):
        if self.on_packet is not None:
            if self._start_streaming:
                self._deliver_promise(b"Ok")
                self._start_streaming = False

            self.on_packet(packet)
        else:
            self._deliver_promise(packet)
        return

    def _on_no_more_data(self, packet):
        if self.on_packet is not None and self._start_streaming:
            self._deliver_promise(b"Ok")
            self._start_streaming = False

    # def _on_event(self, event):
    #     logger.info(event)

    #     if self.event_future is not None:
    #         future, self.event_future = self.event_future, None
    #         future.set_result(event)

    #     if self.on_event:
    #         self.on_event(event)

    def _on_error(self, response):
        logger.debug("Error: %s", response)
        if self._start_streaming:
            self.set_on_frame(None)
        try:
            future = self.request_queue.pop()
            future.set_exception(XSensException(response))
        except IndexError as e:
            raise XSensException(response) from e

    def connection_lost(self, exc):
        self.transport = None
        logger.info("Disconnected")
        if self.on_disconnect is not None:
            self.on_disconnect(exc)


class NSConnection(object):
    """Represent a connection to XSens Network Streamer.

        Returned by :func:`~xsens.connect` when successfully connected to XSens Network Streamer.
    """

    def __init__(self, protocol: XSensProtocol, timeout):
        super(NSConnection, self).__init__()
        self._protocol = protocol
        self._timeout = timeout

    def disconnect(self):
        """Disconnect from QTM."""
        self._protocol.transport.close()

    def has_transport(self):
        """ Check if connected to QTM """
        return self._protocol.transport is not None

    # async def stream_frames(self, on_frame=None):
    #     """Stream measured frames streamed by XSense Network Streamer
    #     """

    #     self._protocol.set_on_frame(on_frame)

    #     cmd = "streamframes %s %s" % (frames, " ".join(components))
    #     return await asyncio.wait_for(
    #         self._protocol.send_command(cmd), timeout=self._timeout
    # )


async def connect(
    host, port=9763, transport_protocol="udp", on_disconnect=None, timeout=5, loop=None,
) -> NSConnection:
    """Async function to connect to Xsens NS

    :param host: Address of the computer running NetWork Streamer.
    :param port: Port number to connect to. Default is 9763.
    :param transport_protocol: Protocol to use. Default is UDP.
    :param on_disconnect: Function to be called when a disconnect from XSense Network Streamer occurs.
    :param timeout: The default timeout time for calls to XSense Network Streamer.
    :param loop: Alternative event loop, will use asyncio default if None.

    :rtype: A :class:`.NSConnection`
    """
    loop = loop or asyncio.get_event_loop()

    try:
        if transport_protocol == "udp":
            _, protocol = await loop.create_datagram_endpoint(
                lambda: XSensProtocol(loop=loop, on_disconnect=on_disconnect),
                remote_addr=(host, port),
            )
        else:
            _, protocol = await loop.create_connection(
                lambda: XSensProtocol(loop=loop, on_disconnect=on_disconnect),
                host,
                port,
            )
    except (TimeoutError, OSError) as exception:
        logger.error(exception)
        return None

    return NSConnection(protocol, timeout=timeout)

