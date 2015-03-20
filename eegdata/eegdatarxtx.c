#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include "eegdatarxtx.h"


#define XON 0x11
#define XOFF 0x13
#define START 0x12
#define STOP 0x14
#define TESTMODE 0x07





void receivedata(const char mode){
	int length=PACKETSIZE+2;//8 bytes of data with start and stop bytes
	char control;
	char packet[PACKETSIZE*2];	
	command[0]='\0';


	int result=write(port, &mode, 1);
	tcdrain(port);
	if(result!=1){
	  perror("write start");
	  close(port);
	  return;    
	  
	}
	
	while(!strlen(command)){
		
	  result = read(port,packet,length);
	  
	  if(result==length){
			if(packet[0]==XON && packet[length-1]==XOFF){
					unsigned short data=((unsigned short)packet[1]<<8)+packet[2];
					cb_push_back(&rxbuffer,&data);
					data=((unsigned short)packet[3]<<8)+packet[4];
					cb_push_back(&rxbuffer,&data);
					data=((unsigned short)packet[5]<<8)+packet[6];
					cb_push_back(&rxbuffer,&data);
					data=((unsigned short)packet[7]<<8)+packet[8];
					cb_push_back(&rxbuffer,&data);
			}
			else{
				printf("start=%d ,stop=%d \n",packet[0],packet[length-1]);
				fprintf(stderr,"read failed\n");
				while(packet[0]!=XOFF){
					read(port,packet,1);//wait for stop			
				}		
			}
	  }
	  
	}
	control=STOP;
	result=write(port, &control, 1);
	tcdrain(port);
	if(result!=1){
	perror("write stop");
	}
}


void transfercommand(char* c){
	int length=strlen(c);
	if(length){
		int result=write(port,c,length);
		tcdrain(port);
		if(result!=length)fprintf(stderr,"write command failed\n");
	}	
	command[0]='\0';
}


void* portio(void* arg){

  

	while(1){
		printf("received command %s\n",command);
		switch(command[0]){
			case 'f':
				receivedata(TESTMODE);
				break;
			case 'd':
				receivedata(START);						
				break;
			case 's'://stop
				command[0]='\0';
				while(!strlen(command)){usleep(2000);}	
				break;
			case 'x': 
				command[0]='\0';
				return 0;
			default:
				transfercommand(command);
				while(!strlen(command)){usleep(2000);}	
				break;
				}		
		
	}
	
	


}
