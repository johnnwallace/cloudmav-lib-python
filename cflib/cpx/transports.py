import socket
import struct
import ssl
import websocket
from threading import Lock

from . import CPXPacket

found_serial = True
try:
    import serial
except ImportError:
    found_serial = False


class CPXTransport:
    def __init__(self):
        raise NotImplementedError('Cannot be used')

    # Change this to URI?
    def connect(host, port):
        raise NotImplementedError('Cannot be used')

    def disconnect():
        raise NotImplementedError('Cannot be used')

    def send(self, data):
        raise NotImplementedError('Cannot be used')

    def receive(self, size):
        raise NotImplementedError('Cannot be used')


class SocketTransport(CPXTransport):
    def __init__(self, host, port):
        print('CPX socket transport')
        self._host = host
        self._port = port

        self.connect()

    def connect(self):
        print('Connecting to socket on {}:{}...'.format(self._host, self._port))
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((self._host, self._port))
        print('Connected')

    def disconnect(self):
        print('Closing transport')
        self._socket.shutdown(socket.SHUT_WR)
        self._socket.close()
        self._socket = None

    def writePacket(self, packet):
        data = bytearray(struct.pack('H', packet.length+2))
        data += packet.wireData
        self._socket.send(data)

    def _readData(self, size):
        data = bytearray()
        while len(data) < size and self._socket is not None:
            data.extend(self._socket.recv(size-len(data)))
        return data

    def readPacket(self):
        size = struct.unpack('H', self._readData(2))[0]

        data = self._readData(size)

        packet = CPXPacket()
        packet.wireData = data

        return packet

        def __del__(self):
            print('Socket transport is being destroyed!')


class UARTTransport(CPXTransport):
    def __init__(self, device, baudrate):
        print('CPX UART transport')
        self._device = device
        self._baudrate = baudrate
        self._serial = None
        self._cts = False
        self._lock = Lock()

        self.connect()

    def connect(self):
        print('Connecting to UART on {} @ {}...'.format(self._device, self._baudrate))
        self._serial = serial.Serial(self._device, self._baudrate, timeout=None)

        isInSync = False

        while not isInSync:
            start = self._serial.read(1)[0]
            print(start)
            if start == 0xFF:
                print('Got start')
                size = self._serial.read(1)[0]
                print(size)
                if size == 0x00:
                    isInSync = True
                    self.cts = True

        # Send back sync
        self._serial.write([0xFF, 0x00])

        print('Connected')

    def _calcXORchecksum(self, data):
        checksum = 0
        for i in data:
            checksum ^= i
        return checksum

    def disconnect(self):
        print('Closing transport')
        self._serial.close()
        self._serial = None

    def writePacket(self, packet):
        self._lock.acquire()
        data = packet.wireData
        if len(data) > 100:
            raise 'Packet too large!'

        buff = bytearray([0xFF, len(data)])
        buff.extend(data)
        buff.extend([self._calcXORchecksum(buff)])
        self._serial.write(buff)

    def readPacket(self):
        size = 0
        while size == 0:
            start = self._serial.read(1)[0]
            if start == 0xFF:
                size = self._serial.read(1)[0]
                if size == 0:
                    self._lock.release()
                else:
                    data = self._serial.read(size)  # Size is excluding start (0xFF) and checksum at end
                    crc = self._serial.read(1)
                    # CRC includes start and size
                    calculated_crc = self._calcXORchecksum(bytes([start, size]) + data)
                    if calculated_crc != ord(crc):
                        print('CRC error!')
                    # Send CTS
                    self._serial.write([0xFF, 0x00])

                    packet = CPXPacket()
                    packet.wireData = data

        return packet


class WebSocketTransport(CPXTransport):
    def __init__(self, url, ca_cert_path):
        print('CPX websocket transport')
        self._url = url
        self._ws = None
        self._ca_cert_path = ca_cert_path

        self.connect()

    def connect(self):
        print('Connecting to websocket on {}...'.format(self._url))

        # Configure SSL for TLS
        sslopt = {}
        if self._url.startswith('wss://'):
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
            ssl_context.check_hostname = False # For testing
    
            if self._ca_cert_path:
                ssl_context.load_verify_locations(self._ca_cert_path)

            sslopt = {'context': ssl_context}

        # Create WebSocket connection with TLS if enabled
        self._ws = websocket.create_connection(
            self._url,
            sslopt=sslopt
        )
        print('Connected')

    def disconnect(self):
        print('Closing transport')
        if self._ws:
            self._ws.close()
        self._ws = None

    def writePacket(self, packet):
        data = bytearray(struct.pack('H', packet.length+2))
        data += packet.wireData
        self._ws.send_binary(data)

    def readPacket(self):
        data = self._ws.recv()

        # Handle binary frame
        if isinstance(data, bytes):
            size = struct.unpack('H', data[:2])[0]
            packet_data = data[2:2+size]
            
            packet = CPXPacket()
            packet.wireData = packet_data
            
            return packet
        
        return None

    def __del__(self):
        print('Websocket transport is being destroyed!')

class CRTPTransport(CPXTransport):
    def __init__(self):
        print('CPX CRTP transport')

    # This connection will not really work...
    def connect(host, port):
        pass

    def disconnect():
        pass

    def send(self, data):
        pass

    def receive(self, size):
        pass
