from msp import MSP_Client_Serial, MSP_RX, MSP_Packet, MSP_Command
from argparse import ArgumentParser
from serial.tools import list_ports
import sys


class A(MSP_RX):

    def receive_packet(self, packet: MSP_Packet) -> None:
        print(packet)


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-p', '--port', type=str, default=None)
    parser.add_argument('-b', '--baud', type=int, default=115200)
    args = parser.parse_args()

    if args.port is None:
        available_ports = [port.device for port in list_ports.comports()]
        if not available_ports:
            print('No serial port chosen, and no serial ports found!')
            sys.exit(0)

        if len(available_ports) == 1:
            port = available_ports[0]
            print(f'One serial port detected: {port}')
        else:
            print(f'No serial port selected, found {len(available_ports)} ports:')
            for i, port in enumerate(available_ports):
                print(f'  {i}: {port}')
            port_sel = int(input('> Choose which port to connect to: ').strip())

            port = available_ports[port_sel]
    else:
        port = args.port


    client = MSP_Client_Serial(port, args.baud)
    client.start()
    resp = client.send_and_receive(MSP_Command.MSP_BOARD_INFO)
    print(resp.data)
    client.stop()