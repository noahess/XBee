import numpy as np
import serial
from serial.tools import list_ports
from typing import Union
from xbee.xbee_templates import Templates
import time


class XBee:
    def __init__(self, com_port: Union[serial.Serial, str], baud=None):
        assert isinstance(com_port, serial.Serial) or isinstance(com_port, str)

        # Create connection
        self.conn = com_port if isinstance(com_port, serial.Serial) else serial.Serial(com_port, baudrate=baud)

        # Pull templates
        self.templates = Templates()

        # Set connection properties
        self.conn.timeout = 0.100

        # Set frame delay
        self.last_message = time.time()
        self.frame_delay = 0.010
        self.motor_unblocked = 0

        # Wait for connection to open
        while not self.conn.writable() or not self.conn.readable():
            pass

    def __del__(self):
        self.conn.close()

    @staticmethod
    def get_connected() -> list:
        """
        Get connected XBees
        :return: List of connected XBees
        :rtype: list
        """
        xbees = []

        # Iterate through comports
        for port in list_ports.comports():

            # Check if device identifies as an XBee
            if port.vid == 1027 and port.pid == 24577:
                xbees.append([port.device, port.serial_number])
        return xbees

    @staticmethod
    def auto_configure():
        """
        Automatically configures a connected XBee
        :return: XBee class
        :rtype: XBee
        """
        xbees = XBee.get_connected()
        return XBee(xbees[0][0], 115200)

    @property
    def read_available(self) -> bool:
        """
        Check if bytes available to read
        :return: True if bytes available else false
        :rtype: bool
        """
        return bool(self.conn.in_waiting)

    @staticmethod
    def _validate_checksum(message: bytes) -> bool:
        """
        Validate incoming checksum
        :param message: The incoming message to check (excluding frame delimiter, length bytes, but including checksum)
        :type message: bytes
        :return: True if the checksum is valid else False
        :rtype: bool
        """
        checksum = 0xFF - (np.frombuffer(message[:-1], dtype='uint8').sum() & 0xFF)
        return checksum == message[-1]

    @staticmethod
    def _generate_checksum(message: bytes) -> bytes:
        """
        Generates the XBee checksum for a given message
        :param message: The message
        :type message: bytes
        :return: Checksum
        :rtype: bytes
        """
        return bytes([0xFF - (np.frombuffer(message, dtype='uint8').sum() & 0xFF)])

    @staticmethod
    def _escape(message: bytes) -> bytes:
        """
        Escape a given message
        :param message: The message to escape
        :type message: bytes
        :return: The escaped message
        :rtype: bytes
        """
        escaped = b''  # The buffer to return

        # Iterate over the given message
        for byte in message:

            # Check if byte needs to be excaped
            if byte == 0x7E or byte == 0x7D or byte == 0x11 or byte == 0x13:
                escaped += b'\x7D' + bytes([byte ^ 0x20])  # Do escape
            else:
                escaped += bytes([byte])  # Don't escape
        return escaped

    def _read_if_available(self) -> bytes:
        """
        Reads an XBee message over serial
        :return: The message (if message available)
        :rtype: bytes
        """
        if self.conn.in_waiting:  # Check if message available
            if self.conn.read() != bytes([0x7E]):  # Assert the first byte is a frame delimiter
                raise Exception("Bytes lost")
            length_high = self.conn.read()  # Read high length byte
            length_low = self.conn.read()  # Read low length byte
            msg_length = (length_high[0] << 8) + length_low[0]  # Compute length

            msg = (msg_length + 1) * [0]  # Prepare buffer
            for byte_count in range(msg_length + 1):  # Iterate for message and checksum bytes
                now = time.time()  # Loop timeout condition
                while time.time() < now + self.conn.timeout and not self.conn.in_waiting:
                    pass
                if not self.conn.in_waiting:  # Check if serial timed out
                    raise Exception("Serial Timeout")
                msg[byte_count] = int(self.conn.read()[0])  # Read byte
                if msg[byte_count] == 0x7D:  # If escape byte, do escape
                    msg[byte_count] = int(self.conn.read()[0]) ^ 0x20

            return bytes(msg)

        return b""  # Return nothing if nothing available

    def read(self, explain=False) -> Union[dict, list]:
        """
        Read an XBee message if available
        :param explain: Returns dictionary with fields if True else returns list when False
        :type explain: bool
        :return: Message in list format if explain is False else message in dictionary format
        :rtype: Union[dict, list]
        """
        msg = self._read_if_available()  # Read message

        if msg:
            if not self._validate_checksum(msg):  # Check checksum
                raise Exception("Checksum incorrect")
            else:
                if explain:  # Return dict if explain is True
                    return self.templates.explain_message(msg[:-1])
                else:
                    return self.templates.unpack_message(msg[:-1])
        else:
            return []  # If no bytes available, return nothing

    def _prepare_write(self, message: bytes) -> bytes:
        """
        Prepares message for writing
        :param message: The message to prepare
        :type message: bytes
        :return: A valid XBee frame
        :rtype: bytes
        """
        length = len(message)  # Calculate length of message
        l_upr = length >> 8 & 0xFF  # Calculate length high byte
        l_lwr = length & 0xFF  # Calculate length low byte
        frame = bytes([0x7E, l_upr, l_lwr])  # Calculate first three bytes of frame

        # Escape message and checksum
        msg_complete = self._escape(message + self._generate_checksum(message))
        frame += msg_complete  # Add escaped message to frame
        return frame

    def write(self, *args) -> bool:
        """
        Writes a message to the connected XBee given frame fields
        :param args: Frame fields from matching XBee template
        :return: True if message succeeded
        :rtype: bool
        """
        if not self.templates.can_process(bytes([args[0]])):  # Assert API ID is recognized
            raise Exception('Unrecognized message!')
        packed = self.templates.pack_message(*args)  # Pack arguments into a message
        msg = self._prepare_write(packed)  # Prepare message for sending

        # Delay until ready to send next packet
        while time.time() < max(self.last_message + self.frame_delay, self.motor_unblocked):
            pass

        self.conn.write(msg)  # Send prepared frame

        self.last_message = time.time()  # Update last message time

        self.conn.flush()  # Wait for output to complete

        return True

    def transmit_request(self, frame_id: int, destination: int, radius: int, options: int, data: bytes):
        """
        Sends a transmit request
        :param frame_id: The frame ID
        :type frame_id: int
        :param destination: The 64 bit destination address
        :type destination: int
        :param radius: The broadcast radius
        :type radius: int
        :param options: The transmit options
        :type options: int
        :param data: The rf data
        :type data: bytes
        :return: None
        """
        self.write(0x10, frame_id, destination, 0xFFFE, radius, options, data)

    def read_until_frame_id(self, target_id: int) -> list:
        """
        Read until a frame ID is reached
        :param target_id: The frame ID to stop reading at
        :type target_id: int
        :return: The list of frames read until the ID was reached
        :rtype: list
        """
        start = time.time()  # Set timeout condition
        output_frames = []  # Initialize result
        frame_id = 0  # Set current frame id
        while time.time() < start + self.conn.timeout and target_id != frame_id:
            if self.read_available:  # Read if bytes available
                this_frame = self.read()
                frame_id = this_frame[1]  # Obtain frame ID
                output_frames.append(this_frame)
        return output_frames

    def discard_until_frame_id(self, target_id: int, except_on_not_status=False) -> list:
        """
        Reads and discards until frame ID is reached
        :param target_id: The frame ID to stop reading at
        :type target_id: int
        :param except_on_not_status: Raise exception when a non-status packet is read with frame id != target id
        :type except_on_not_status: bool
        :return: The resulting frame
        :rtype: list
        """
        result = self.read_until_frame_id(target_id)  # Gather frames
        if result:
            if except_on_not_status:  # Raise exception if required
                for i in range(len(result) - 1):
                    if result[i][0] != 0x8B and result[i][1] != target_id:
                        raise Exception("Discarded valid packet!")
            return result[-1]
        return []

    def queue_multiple_messages(self, frame_ids: Union[list, int], destination: int, radius: int, options: int,
                                messages: list) -> None:
        """
        Sends multiple transmit requests
        :param frame_ids: The frame IDs or single frame ID to be applied to all
        :type frame_ids: Union[list, int]
        :param destination: The 64 bit destination address
        :type destination: int
        :param radius: The broadcast radius
        :type radius: int
        :param options: The transmit options
        :type options: int
        :param messages: The list of rf data
        :type messages: list
        :return: None
        """
        if not isinstance(frame_ids, int):  # Assert frame ids and messages are same size
            assert len(messages) == len(frame_ids)
        for index in range(len(messages)):  # Iterate over messages and send
            self.transmit_request(frame_id=frame_ids if isinstance(frame_ids, int) else frame_ids[index],
                                  destination=destination, radius=radius, options=options,
                                  data=messages[index])

    def clear_incoming(self) -> None:
        """
        Clears the entire input buffer
        :return: None
        """
        self.conn.reset_input_buffer()
