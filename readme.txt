TRI86 EV driver controls MSP430 firmware README

- This firmware is written to use the GNU GCC3 toolchain, please read the install instructions below.
- Refer to the board schematics, available on the Tritium website, for pinouts and device functionality.
- The firmware is licenced using the BSD licence.  There is no obligation to publish modifications.
- If you've written something cool and would like to share, please let us know about it!
- Contact James Kennedy with any questions or comments: james@tritium.com.au



Toolchain walkthrough (Windows MSPGCC instructions):

- Download and install the 2008-12-30 release of MSPGCC3:
  http://sourceforge.net/projects/mspgcc/files/Outdated/mspgcc-win32/snapshots/mspgcc-20081230.exe/download
  The Tritium bootloader tool requires this specific version of GCC, as it has not yet been updated to use the
  output from the current GCC4 tools.

- Download and install the current MinGW, and use the install options for the C compliler + MSYS.  This gets you 'make', 'rm', etc 
  http://sourceforge.net/projects/mingw/files/

- Set your path (system -> advanced -> environment variables -> path -> edit or similar) to include the three things above:  
  c:\mspgcc\bin; c:\MinGW\bin; c:\MinGW\msys\1.0\bin; or whatever path you installed them to. 

- Open a cmd prompt window, change to the directory with the firmware files, and type: make
  This should compile everything and produce an .a43 and .elf file ouput

- Still at the command line, type: encrypt
  This should take the .a43 and encryption key and produce a Tritium bootloader .tsf file targetted at the driver controls hardware
  You may need to adjust the version number in the encrypt.bat file to match your driver controls hardware version number

- Use the triFwLoad tool from the Tritium website, with a CAN base address of 0x500, to bootload the .tsf file into the driver controls over the CAN bus
