#ifndef EEGDATARXTX_H
#define EEGDATARXTX_H

#include "circular_buffer.h"


#define PACKETSIZE 8 //packet is 8 bytes or 4 short int

circular_buffer rxbuffer;	
char command[8];
int port;
void* portio(void* arg);

#endif
