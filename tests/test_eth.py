import socket
import sys

sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
sock.bind(('eth0', 0))

# Ethernet header
dest_mac = b'\xff\xff\xff\xff\xff\xff'
src_mac = b'\x2c\xcf\x67\x6e\x72\xf4'
ethertype = b'\x88\xb5' # Local experimental (0x0800 = IPv4, 0x0806 = ARP, 0x86DD = IPv6)

payload = f"motor {sys.argv[1]}\n".encode()

packet = dest_mac + src_mac + ethertype + payload
sock.send(packet)