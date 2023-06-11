from abc import abstractmethod, ABC
from dataclasses import dataclass, field
from queue import Queue
from serial import Serial, SerialTimeoutException
import struct
from enum import IntEnum
from threading import Thread, Event
from copy import copy


class MSP_TX(ABC):

    @abstractmethod
    def send(self, data: bytes) -> None:
        pass

@dataclass
class MSP_Packet:
    size: int = 0
    command: int = 0
    data: bytes = field(default_factory=bytearray)
    crc: int = 0
    raw: bytearray = field(default_factory=bytearray)


class MSP_RX(ABC):

    @abstractmethod
    def receive_packet(self, packet: MSP_Packet) -> None:
        pass

class MSP_Command(IntEnum):
    MSP_API_VERSION 	  = 1
    MSP_FC_VARIANT 	      = 2
    MSP_FC_VERSION 	      = 3
    MSP_BOARD_INFO        = 4
    MSP_BUILD_INFO        = 5
    MSP_IDENT             = 100
    MSP_RAW_IMU           = 102
    MSP_MOTOR             = 104
    MSP_SET_MOTOR         = 214
    MSP_ATTITUDE          = 108
    MSP_PID               = 112
    MSP_SET_PID           = 202
    # CUSTOM
    MSP_READ_MEM_ADDRESS  = 180
    MSP_WRITE_MEM_ADDRESS = 181


def calculate_crc(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
    return crc

def hex_string(data: bytes) -> str:
    return ' '.join(hex(a)[2:].zfill(2) for a in data)

class MSP_StateMachine:

    def __init__(self, rx: MSP_RX) -> None:
        self._malformed_packets = 0
        self._rx = rx
        self._bytes_read = 0
        self._rx_buf = bytearray()
        self._crc = 0
        self._packet = MSP_Packet()

    def process_byte(self, byte: int) -> None:
        bytes_read = len(self._rx_buf)
        self._rx_buf.append(byte)

        if bytes_read == 0:                   # Preamble byte 1
            if byte != ord(b'$'):
                print('ASD1')
        elif bytes_read == 1:                 # Preamble byte 2
            if byte != ord(b'M'):
                print('ASD2')
        elif bytes_read == 2:                 # Direction
            if byte != ord(b'>'):
                print('ASD3')
        elif bytes_read == 3:                 # Size
            self._packet.size = byte
            self._crc = byte
        elif bytes_read == 4:                 # Command
            self._packet.command = byte
            self._crc ^= byte
        elif (bytes_read - 4) <= self._packet.size:  # Data
            self._packet.data.append(byte)
            self._crc ^= byte
        else:                                       # CRC
            self._packet.crc = byte

            crc_calculation_buf = self._rx_buf[3:-1]
            crc = calculate_crc(crc_calculation_buf)

            # Validate packet
            packet_ok = True
            if crc != self._packet.crc:
                print(f'Incorrect CRC, got {self._packet.crc}, expected {crc}')
                packet_ok = False
                self._malformed_packets += 1

            if packet_ok:
                self._packet.raw = copy(self._rx_buf)
                self._rx.receive_packet(copy(self._packet))

            # Reset
            self._crc = 0
            self._rx_buf = bytearray()
            self._packet = MSP_Packet()


class MSP_Client(MSP_RX):
    header = b'$M<'
    read_timeout = 1

    def __init__(self, callback: MSP_RX = None) -> None:
        self._rx_packets = Queue()
        self._is_running = Event()
        self.msp_fsm = MSP_StateMachine(self)
        self._rx_callback = callback

    def receive_packet(self, packet: MSP_Packet) -> None:
        print(f'RX: (len={len(packet.raw)}) {hex_string(packet.raw)}')
        self._rx_packets.put(packet)
        if self._rx_callback:
            self._rx_callback.receive_packet(packet)

    def _read_thread(self) -> None:
        print('Read thread starting')
        while self._is_running.is_set():
            byte = self.read_one_byte()
            if byte:
                self.msp_fsm.process_byte(ord(byte))

        print('Read thread ending')
        self.disconnect()

    def send_and_receive(self, command: MSP_Command, data: bytes = b'', timeout: int = 2) -> MSP_Packet:
        self.send(command, data)
        try:
            return self._rx_packets.get(timeout=timeout)
        except Exception as e:
            print(e)
            return None

    def send(self, command: MSP_Command, data: bytes = b'') -> bytes:
        len_command_data = struct.pack('BB', len(data), command) + data
        crc = calculate_crc(len_command_data)
        tx_pkt = self.header + len_command_data + struct.pack('B', crc)

        print(f'TX (len={len(tx_pkt)}) {hex_string(tx_pkt)}')
        self.do_send(tx_pkt)

    def start(self) -> None:
        if not self.is_connected():
            self.connect()

        self._is_running.set()
        Thread(target=self._read_thread, daemon=True).start()
        
    def stop(self) -> None:
        self._is_running.clear()

    def connect(self) -> bool:
        if self.is_connected():
            print('MSP Client already connected, not connecting again...')
            return False

        self.do_connect()
        print(f'Connected to {self.info()}')
        return True

    def disconnect(self) -> bool:
        if not self.is_connected():
            print('MSP Client not connected, not disconnecting...')
            return False

        self.do_disconnect()
        return True

    def read_one_byte(self) -> bytes:
        try:
            return self.do_read_one_byte()
        except TimeoutError:
            return None

    @abstractmethod
    def info(self) -> str:
        pass

    @abstractmethod
    def do_read_one_byte(self) -> bytes:
        pass

    @abstractmethod
    def do_connect(self) -> None:
        pass

    @abstractmethod
    def do_disconnect(self) -> None:
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        pass

    @abstractmethod
    def do_send(self, data: bytes) -> int:
        pass


class MSP_Client_Serial(MSP_Client):

    def __init__(self, port: str, baudrate: int, callback: MSP_RX = None) -> None:
        super().__init__(callback)
        self.port = port
        self.baudrate = baudrate
        self._serial: Serial = None

    def do_connect(self) -> None:
        self._serial = Serial(self.port, self.baudrate, timeout=self.read_timeout)

    def do_disconnect(self) -> None:
        self._serial.close()

    def do_send(self, data: bytes) -> int:
        self._serial.write(data)
        self._serial.flush()

    def do_read_one_byte(self) -> bytes:
        return self._serial.read(1)

    def info(self) -> str:
        return f'Serial port: {self.port}, baud: {self.baudrate}'

    def is_connected(self) -> bool:
        return self._serial is not None
