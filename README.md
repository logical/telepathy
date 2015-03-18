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

The firmware is for a PIC microcontroller 16f1788.The microcontroller first sets up the hc-05,
then wait for a request.When it receives  a request it begins sampling tha analog to digital converter
and sending the samples to the hc-05. I chose th 16f1788 but the code should be easy to modify to work
with another PIC.

I have also included a mcp4131-103 digital pot to adjust the final amplifier.


SCHEMATIC:

draft.png is a rough draft of my current design which is under development and will certainly change.The multiplexed design may be difficult but it is just an experiment. the npn transistors ground the unselected inputs to reduce parasitics.

The analog amplifier portion of the circuit is based on [this dual channel fm frontend from openeeg](http://openeeg.sourceforge.net/doc/hw/sceeg/DualChannelFMUnit-Sheet1.jpg) 





Disclaimer:

This project is an EXPERIMENT. I make no claims that it actually does anything.It may be harmful to 
individuals or property.
THIS DEVICE AND SOFTWARE MAY BE DANGEROUS. DO NOT BUILD IT.
