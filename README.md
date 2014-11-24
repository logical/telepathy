This is telepathy the MOSTLY hardware project not telepathy the software project
Sorry for my lack of creativity in choosing a name.

This poject contains a bluetooth eeg circuit based on openeeg dual channel sound card eeg project.

It contains.


EEGDATA:

    A software program for linux that retreives the data from the /dev/rfcomm0 bluetooth serial terminal
    and analyzes the data using opencv. Currently it does a discrete fourier transform on the data to get 
    a spectrum analysis. It can save signal data to eegdatasettings file and compare them to the live signal
    using signal to noise ratio and similarity meaurement.
    I started the project in C but it was more convenient to write the opencv portion in C++. 








Disclaimer:

This project is an EXPERIMENT. I make no claims that it actually does anything.It may be harmful to individuals or property.
THIS DEVICE AND SOFTWARE MAY BE DANGEROUS. DO NOT BUILD IT.
