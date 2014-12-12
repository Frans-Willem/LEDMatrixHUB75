Compiling OpenWRT as our Art-Net bridge
===
Why
---
The STM32F4 Discovery board has built-in ethernet hardware, but no ethernet port. Furthermore I wanted to be able to hang this thing somewhere with just a power cable attached, so I preferred WiFi. I decided to use a cheap OpenWRT compatible router, the TP-Link WR703N, to receive Art-Net data over WiFi, and spit it out on it's serial line.

Notes:
- To bring out the WR703N's serial port, google, or use this guide: http://blog.lincomatic.com/?p=1229
- To get the desired framerate, I had to patch the UART driver to allow higher baudrates.
- If you can't get this exact router, try to find one with similar hardware, especially the same UART driver. Chances are all code will work as is.
- The net2ledmatrix package is really simple, and uses only basic Linux calls. It should work on other systems too, provided the serial port can handle 5MBaud.

Steps
---
- Clone this github repository somewhere. I will assume it's cloned at ~/LEDMatrixHUB75 from now on.
- Install OpenWRT Buildroot (http://wiki.openwrt.org/doc/howto/buildroot.exigence)
    - sudo apt-get update
    - sudo apt-get install git-core build-essential subversion
    - git clone git://git.openwrt.org/openwrt.git
    - cd openwrt
- We don't need all OpenWRT packages available, base packages and LuCI related ones are enough. (https://forum.openwrt.org/viewtopic.php?id=16599)
    - ./scripts/feeds update packages luci
    - ./scripts/feeds install -a -p luci
- Link in our Net2LEDMatrix package
    - ln -s ~/LEDMatrixHUB75/net2ledmatrix/ package/utils/net2ledmatrix
- Link in our AR933x patch
    - ln -s ~/LEDMatrixHUB75/000-ar933x_uart_speed.patch target/linux/ar71xx/patches-3.14/
- Configure which router to compile for
    - make menuconfig
    - Go into "Target profile" and pick TP-LINK TL-WR703N
    - Go into "LuCI" -> "Collections" and enable luci (hit space twice so it gets a * instead of M).
    - Go into "Utilities" and enable net2ledmatrix with a * to preinstall it, or with an M to compile as a package.
    - Save & Exit
- Disable the serial console (because we'll be using it to communicate with the LED Matrix instead)
    - Open target/linux/ar71xx/image/Makefile
    - Find the line containing TL-WR703N (or whichever other router you use)
    - Replace ttyATH0 with ttyS0
    - It should now read something along the lines of:
   (call SingleProfile,TPLINK-LZMA,64kraw,TLWR703,tl-wr703n-v1,TL-WR703N,ttyS0,115200,0x07030101,1,4Mlzma))
- Start the compilation process
    - make
- Wait. It could take a few hours, or days.
- Your binary image should be in bin/ar71xx.
- If you didn't choose to bake in the proxy in the firmware image, you should look in bin/ar71xx/packages/base for the net2ledmatrix package, and install that manually on the router.