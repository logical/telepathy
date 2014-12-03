This is telepathy the mostly HARDWARE project not telepathy the software project
Sorry for my lack of creativity in choosing a name.

This poject contains a bluetooth eeg circuit based on openeeg dual channel sound card eeg project.

GOAL
------
The goal is to do brain computer interface experments.I want to make a wireless headband that runs
on a 9v battery. I will design the board with kicad using surface mount parts.I plan on using a
computer numerical controller machine to cut 2 one sided boards and stick them together back to back.

It might not work but that's the plan!



It contains.


EEGDATA:

A software program for linux that retreives the data from the /dev/rfcomm0 bluetooth serial terminal
and analyzes the data using opencv. Currently it does a discrete fourier transform on the data to get 
a spectrum analysis. It can save signal data to eegdatasettings file and compare them to the live signal
using signal to noise ratio and similarity meaurement.
I started the project in C but it was more convenient to write the opencv portion in C++. 



FIRMWARE:

The firmware is for a PIC microcontroller 16f1788.All the microcontroller does is first set up the hc-05,
then wait for a request.When it receives  a request it begins sampling tha analog to digital converter
and sending the samples to the hc-05. I chose the 16f1788 but the code should be easily modified to work
with another PIC.


SCHEMATIC:

Still in development but based on [this dual channel fm frontend from openeeg](http://openeeg.sourceforge.net/doc/hw/sceeg/DualChannelFMUnit-Sheet1.jpg) 





Disclaimer:

This project is an EXPERIMENT. I make no claims that it actually does anything.It may be harmful to 
individuals or property.
THIS DEVICE AND SOFTWARE MAY BE DANGEROUS. DO NOT BUILD IT.
