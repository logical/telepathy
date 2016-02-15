This is telepathy the mostly HARDWARE project not telepathy the software project
Sorry for my lack of creativity in choosing a name.

This poject contains an EXPERIMENTAL bluetooth eeg circuit.

GOAL
------
This project is under development so it is not working. I welcome suggestions.


The goal is to do brain computer interface experments.I want to make a wireless headband that runs
on a battery. 

It contains.

STATUS
------
I am currently doing a complete redesign of the hardware.Here are some of the changes I'm planning.

1. Split the device into 3 sections for power, amplifier and input/output.
2. Replace the bluetooth module and pic with a bluetooth capable microcontroller.
3. add active electrodes.

ELECTRODES: 

I have 2 designs for active electrodes. Active dry electrodes and active noncontact electrodes.
The dry electrodes are based on the openeeg design. The noncontact electrodes are based on a design I found here:

http://www.isn.ucsd.edu/pubs/bsn10.pdf

I'm assuming it is creative commons.



EEGDATA:

A GUI software program for linux that retreives the data from the /dev/rfcomm0 bluetooth serial terminal
and analyzes and displays the data. I started the project in C but it was more convenient to write the data analysis portion in C++. 

On my system I use these commands to setup the bluetooth serial port.

 	#find the address
 	$hcitool scan
	#bind the address
	$sudo rfcomm bind rfcomm0 device_address
	
For BLE

	#find the adapter address
	$hcitool dev
	#find the device address
	$sudo hcitool -i hci0 lescan
	

Then you need to run any programs that access serial terminal as root unless you have sufficient permission.



FIRMWARE:


The firmware is for a PIC microcontroller 16f1788 and a bluetooth serial module hc-05.

I have wrote a bootloader so I can more easily install firmware.The bootloader sets the hc-05 up at 38400 and waits for a program.
When the program successfully loads it sets a byte in ee memory.when the FLASH command is sent to the program it erases the byte
so the bootloader will again wait for a progam. You can use a standard serial terminal to send the program but you have to set a long delay (~30ms) between characters so the program has time to write. Custom coding the serial interface for the bootloader would be faster but this works for now.

Hopefully I have no glitches in the bootloader or a firmware update could brick the device! 
If so the glitch would have to be fixed and the bootloader reprogrammed.


The microcontroller first sets up the hc-05 at 230400, then wait for a request.When it receives  a request it begins sampling the analog to digital converter and sending the samples to the hc-05.

I have included the makefiles so they should build with the make command but they may require modification . MPLABX should be able to load them as a makefile project but I have not tried.
 

SCHEMATIC:

I'm developing the amplifier board first. amplifier-board.png is the current design.


DISCLAIMER:

This project is an EXPERIMENT. I make no claims that it actually does anything.It may be harmful to 
individuals or property.
THIS DEVICE AND SOFTWARE MAY BE DANGEROUS. DO NOT BUILD IT.
