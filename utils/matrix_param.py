import sys
import struct
import socket

if (len(sys.argv) <= 2):
	print "Usage: %s <IP> <brightness: 0-255> [gamma: 0-255]" % (sys.argv[0])
	sys.exit()
ip = sys.argv[1]
brightness = int(sys.argv[2])
gamma = int(sys.argv[3]) if (len(sys.argv)>3) else None

packet = struct.pack("8s 2B 2B B B 2B 2B",
	"Art-Net",
	0x00,0x50,	#OpOutput
	0,14,		#Version
	0,			#Sequence
	0,			#Physical
	0xFF,0xFF,	#SubUni + Net
	#Length:
	0x00, 1 if (gamma == None) else 2	
	)
packet += struct.pack("B", brightness)
if (gamma != None):
	packet += struct.pack("B", gamma)
	
print "Sending to", ip, repr(packet)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print sock.sendto(packet, (ip, 6454))