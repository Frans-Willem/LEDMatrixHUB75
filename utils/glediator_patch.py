import sys
if (len(sys.argv) <= 3):
	print "Usage: %s <IP> <width> <height>" % (sys.argv[0])
	sys.exit()
ip = sys.argv[1].split(".")
if (len(ip) < 4):
	print "IP should be of the form: xxx.xxx.xxx.xxx"
	sys.exit()
width = int(sys.argv[2])
height = int(sys.argv[3])
pixels = width * height
bytes = pixels * 3
max_bytes_per_universe = 512
pixels_per_universe=max_bytes_per_universe / 3
universes = (pixels + pixels_per_universe - 1) / pixels_per_universe

print "Patch_Matrix_Size_Y=%d" % height
print "Patch_Matrix_Size_X=%d" % width
print "Patch_Num_Unis=%d" % universes
for universeId in range(0,universes):
	print "Patch_Uni_ID_%d_IP1=%s" % (universeId, ip[0])
	print "Patch_Uni_ID_%d_IP2=%s" % (universeId, ip[1])
	print "Patch_Uni_ID_%d_IP3=%s" % (universeId, ip[2])
	print "Patch_Uni_ID_%d_IP4=%s" % (universeId, ip[3])
	print "Patch_Uni_ID_%d_Net_Nr=%d" % (universeId, universeId >> 8)
	print "Patch_Uni_ID_%d_Sub_Net_Nr=%d" % (universeId, (universeId >> 4) & 0xF)
	print "Patch_Uni_ID_%d_Uni_Nr=%d" % (universeId, universeId & 0xF)
	print "Patch_Uni_ID_%d_Num_Ch=%d" % (universeId, min(pixels_per_universe, pixels - (pixels_per_universe * universeId)) * 3 )

for x in range(0, width):
	for y in range(0, height):
		offset = x + (y * width)
		universeId = offset / pixels_per_universe
		universe_offset = offset % pixels_per_universe
		print "Patch_Pixel_X_%d_Y_%d_Uni_ID=%d" % (x,y,universeId)
		print "Patch_Pixel_X_%d_Y_%d_Ch_R=%d" % (x,y,(universe_offset * 3)+0)
		print "Patch_Pixel_X_%d_Y_%d_Ch_G=%d" % (x,y,(universe_offset * 3)+2)
		print "Patch_Pixel_X_%d_Y_%d_Ch_B=%d" % (x,y,(universe_offset * 3)+1)