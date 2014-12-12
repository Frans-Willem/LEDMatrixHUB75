STM32F4Discovery
===
Why
---
Initially I started off with an STM32F1 discovery board, however I soon found out that being able to run at 168mhz would allow me to run the board at a higher bitrate without flickering. Luckily I had an STM32F4 Discovery laying around.

Steps
---
- Download the following prerequisites:
    -  STM32F4 DSP and Standard Peripheral lib. Found at [STMicroelectronics](http://www.st.com/web/catalog/mmc/FM141/SC1169/SS1577/LN11/PF252140?sc=internet/mcu/product/252140.jsp) under 'Design resources', 'Related Tools and Software', 'STSW-STM32065'. [Direct link](http://www.st.com/web/en/catalog/tools/PF257901).
    -  [Eclipse IDE for C/C++ Developers](https://eclipse.org/downloads/packages/eclipse-ide-cc-developers/lunasr1)
    -  [GCC for ARM (take the zip package)](https://launchpad.net/gcc-arm-embedded/+download)
    -  [GNU Make](http://gnuwin32.sourceforge.net/packages/make.htm) (Binaries and dependencies)
    -  [STM32 ST-LINK utility](http://www.st.com/web/en/catalog/tools/PF258168)
- Set up a root directory somewhere where to work from, for example C:\STM32
    - Unzip the STM32F4 DSP and Standard Peripheral Library to C:\STM32\STM32F4xx_DSP_StdPeriph_Lib_V1.4.0
    - Unzip GCC for ARM to C:\STM32\compiler
    - Unzip GNU Make and the dependencies in the C:\stm32\compiler directory too.
- Start up Eclipse, and start a workspace in C:\STM32\Workspace
- File -> Import, Git -> Project from Git, Clone URI
    - Location: https://github.com/Frans-Willem/LEDMatrixHUB75.git
    - Next
    - master, Next
    - Destination: C:\STM32\git
    - Next
    - Import existing projects
    - Next, Finish
- Set to Release build: right-click project -> Build configuration -> Set Active -> Release
- Compile: right-click project -> Build project
- Connect the STM32F4Discovery board to your computer
- Open the STM32 ST-LINK Utility
    - File -> Open File
    - C:\STM32\secondgit\LEDMatrixHUB75\F4Matrix\Release\F4Matrix.bin
    - Target -> Program & Verify
    - Start Address: 0x80000000
    - Check Reset after programming
      - Start
- Congratulations the firmware is now running on your STM32F4Discovery board!