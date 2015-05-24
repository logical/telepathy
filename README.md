This is telepathy the mostly HARDWARE project not telepathy the software project
Sorry for my lack of creativity in choosing a name.

This poject contains a bluetooth eeg circuit based on openeeg dual channel sound card eeg project.

GOAL
------
The goal is to do brain computer interface experments.I want to make a wireless headband that runs
on a battery. I will design the board with kicad using surface mount parts.

It might not work but that's the plan!



It contains.


EEGDATA:

A software program for linux that retreives the data from the /dev/rfcomm0 bluetooth serial terminal
and analyzes the data. 
I started the project in C but it was more convenient to write the data analysis portion in C++. 



FIRMWARE:


The firmware is for a PIC microcontroller 16f1788.

I have wrote a bootloader so I can more easily install firmware.The bootloader sets the hc-05 up at 38400 and waits for a program.
When the program successfully loads it sets abyte in ee memory.when the FLASH command is sent to the program it erases the byte
so the bootloader will again wait for a progam. 

Hopefully I have no glitches in the bootloader or a firmware update could brick the device! 
If so the glitch would have to be fixed and the bootloader reprogrammed.


The microcontroller first sets up the hc-05 at 230400, then wait for a request.When it receives  a request it begins sampling tha analog to digital converter and sending the samples to the hc-05.

I have included a the makefiles so they should build with the make command. MPLABX should be able to load the as a makefile project bt I have not tried.
 

SCHEMATIC:

draft.png is a rough draft of my current design which is under development and will certainly change. the npn transistors ground the unselected inputs to reduce parasitics.

The analog amplifier portion of the circuit is based on [this dual channel fm frontend from openeeg](http://openeeg.sourceforge.net/doc/hw/sceeg/DualChannelFMUnit-Sheet1.jpg) 

I have created a kicad schematic and pcb (kicad build 2012-apr-16-27).The schematic does not have resistor and capacitor values in it yet because I may change them. The pcb is untested. In order to create a pcb you have to export to gerber, then run pcb2gcode on th gerber if needed. 



DISCLAIMER:

This project is an EXPERIMENT. I make no claims that it actually does anything.It may be harmful to 
individuals or property.
THIS DEVICE AND SOFTWARE MAY BE DANGEROUS. DO NOT BUILD IT.
