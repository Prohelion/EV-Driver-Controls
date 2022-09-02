# Prohelion EV Driver Controls (PHLN-3000-0008)

This repository contains the open-source firmware for the Prohelion EV Driver Controls unit. The device can be purchased from the [Prohelion Website](https://www.prohelion.com)

- This firmware is written to use the GNU GCC3 toolchain, please read the install instructions below.
- Refer to the board schematics, available on the Prohelion website, for pin-outs and device functionality.
- The firmware is licensed using the BSD license.  There is no obligation to publish modifications.
- If you've written something cool and would like to share, please let us know about it!

## Toolchain walk-through (Windows MSP GCC instructions):

1) Download and install the 2008-12-30 release of MSPGCC3:
  http://sourceforge.net/projects/mspgcc/files/Outdated/mspgcc-win32/snapshots/mspgcc-20081230.exe/download
  The bootloader tool requires this specific version of GCC..

2) Download and install the current MinGW, and use the install options for the C compiler + MSYS.  This gets you 'make', 'rm', etc 
  http://sourceforge.net/projects/mingw/files/

3) Set your path (system -> advanced -> environment variables -> path -> edit or similar) to include the three things above:  
  c:\mspgcc\bin; c:\MinGW\bin; c:\MinGW\msys\1.0\bin; or wherever you may have installed them. 

4) Open a cmd prompt window, change to the directory with the firmware files, and type: make
  This should compile everything and produce an .a43, .elf a bootloader .tsf file targeted at the driver controls hardware
  ```
  C:\Users\user\EV-Driver-Controls>make
  ```

5) Use the triFwLoad tool, with a CAN base address of 0x500, to bootload the .tsf file into the driver controls over the CAN bus
  ```
  C:\Users\user\EV-Driver-Controls>triFwLoad_2_04.exe build\tri86.tsf
  ```