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
#include <signal.h>

#define MAXFREQUENCY (SAMPLESIZE/4)
#define FIFONAME "./eegdatafifo"
#define DEFAULT_COM	"/dev/rfcomm0"
#define DEFAULT_BAUD	B230400 
#define SAMPLERATE 512
#define PACKETSIZE 4 //packet is 8bytes
#define MAXSAMPLE 65535.0 //16 bits
#define ZEROSAMPLE (MAXSAMPLE/2.0)

#define START 0x12
#define STOP 0x14
#define TEST 0x07;
#define XON 0x11
#define XOFF 0x13

int mode;
int port;
int fifod;

//one byte every 200usx4 is 1/1600us=833sa/s
/*
 * i *nt receive(int port,unsigned char *rxbuffer,unsigned int length){
 * #define TIMEOUT 20
 * unsigned char buffer[length];
 * unsigned char timeout=TIMEOUT;
 * memset(buffer,0,length);
 * int chars=0;
 * while(timeout){
 *  int result = read(port,buffer,length-chars);
 *  if (result >0) {
 *    for(int i=0;i<result;i++){
 *      rxbuffer[i+chars]=buffer[i];
 *      }
 *      chars+=result;
 *      if(chars>=length){
 * //	printf("Receive %d\n",chars);
 * return chars;
 * }
 * }
 * else {
 *   microsleep(400);
 *   timeout--;
 *   }
 *   }
 *   printf("Requested %d only received %d,flushing buffer.\n",length,chars);
 *   tcflush(port,TCIFLUSH);
 *   tcflush(port,TCIFLUSH);
 *   return 0;
 *   
 *   }
 */

/*
void microsleep(unsigned int us){
    struct timespec sleepTime;
    struct timespec returnTime;
    sleepTime.tv_sec = 0;
    sleepTime.tv_nsec = us*10000;//0 to 999999999
    nanosleep(&sleepTime, &returnTime);
}  
*/


// SetupConnection sets the port's baud, parity, etc.
int SetupConnection(int port, int baudRate){
  struct termios attribs;
  memset(&attribs,0,sizeof(attribs));
  cfsetispeed(&attribs, baudRate);//B1500000 
  cfsetospeed(&attribs, baudRate);
  attribs.c_iflag &= ~( IGNCR | IGNBRK | BRKINT | PARMRK | INPCK | ISTRIP | INLCR | ICRNL | IXON | IXOFF );
  attribs.c_iflag=(IGNPAR  | IXANY) ;
  attribs.c_oflag &= ~OPOST;
  attribs.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  attribs.c_cflag &= ~(CSTOPB | CSIZE | PARENB);
  attribs.c_cflag |= CS8;
  attribs.c_cc[VMIN]=PACKETSIZE*2;
  attribs.c_cc[VTIME]=0;
  sleep(2);
  tcflush(port, TCIOFLUSH); 
  tcsetattr(port, TCSANOW, &attribs);
  tcgetattr(port, &attribs);
  return 1;
  
}

void fakepackets(unsigned short *buf){
  
  float freq1 = 10.0;
  float freq2 = 25.0;
  float freq3 = 40.0;
  float freq4 = 55.0;
  static float t=0.0; 
  int scale = 640*4;
  buf[3] = (unsigned short)(sin(2. * M_PI * freq1 * t / scale)*400+ZEROSAMPLE);
  buf[2] = (unsigned short)(sin(2. * M_PI * freq2 * t / scale)*ZEROSAMPLE+ZEROSAMPLE);
  buf[1] = (unsigned short)(sin(2. * M_PI * freq3 * t / scale)*400+ZEROSAMPLE);
  buf[0] = (unsigned short)(sin(2. * M_PI * freq4 * t / scale)*ZEROSAMPLE+ZEROSAMPLE);
  t++;
  if(t>65533)t=0.0;
  usleep(1000);
}

void signal_callback_handler(int signum){
  fprintf(stderr,"Caught signal %d\n",signum);
  // Cleanup and close up stuff here
  
  char control=STOP;
  write(port, &control, 1);
  tcdrain(port);
  close(port);
  
  exit(0);
  
}

void main(int argc,char **argv){
  int opt;
  int result=0;
  unsigned short buffer[PACKETSIZE];
  
  // Register signal and signal handler
  signal(SIGINT, signal_callback_handler);
  
  
  /* write "Hi" to the FIFO */
  while ((opt = getopt(argc, argv, "fdt")) != -1) {
    switch (opt) {
      case 'f': 
	while(1){
	  fakepackets(buffer);
	  //	    fprintf(stderr,"%d\n",buffer[0]);
	  result=fwrite(buffer, sizeof(unsigned short),PACKETSIZE,stdout);
	  //printf("%d %d %d %d\n",buffer[0],buffer[1],buffer[2],buffer[3]);
	  if(result!=PACKETSIZE){
	    fprintf(stderr,"pipe broken:exiting\n");
	    exit(0);
	  }
	}
	break;
      case 'd':
	port = open(DEFAULT_COM, O_RDWR | O_NOCTTY );
	if (port == -1){
	  perror("Opening COM port");
	  exit(0);
	}
	if (!SetupConnection(port, DEFAULT_BAUD)) {
	  perror("SetupConnection");
	  close(port);
	  exit(0);
	}
	char control=START;
	result=write(port, &control, 1);
	if(result!=1){
	  perror("write start");
	  close(port);
	  exit(0);    
	  
	}
	tcdrain(port);
	unsigned char rxbuffer[PACKETSIZE];
	unsigned int timeout=0;
	while(1){

//notice how the unsigned characters read right into the unsigned int position?
//GOT LUCKY == PRETTY SLICK	
	  result = read(port,buffer,PACKETSIZE*2);
	  if (result == PACKETSIZE*2) {
	    timeout=0;
	    
	    result=fwrite(buffer,sizeof(unsigned short),PACKETSIZE,stdout);
	    
	    // fprintf(stderr,"%d %d %d %d \n",buffer[0],buffer[1],buffer[2],buffer[3]);
	    //	      fprintf(stderr,"%d %d %d %d %d %d %d %d \n",rxbuffer[0],rxbuffer[1],rxbuffer[2],rxbuffer[3],rxbuffer[4],rxbuffer[5],rxbuffer[6],rxbuffer[7]);
	    if(result != PACKETSIZE){
	      perror("write pipe");
	      char control=STOP;
	      write(port, &control, 1);
	      tcdrain(port);
	      close(port);
	      exit(0);
	    }
	  }
	  else{
	    // fprintf(stderr,"recieved %d bytes \n",result);
	    //a gap in transmission is used for synchronization	
	    fprintf(stderr,"lost sync\n");
	    tcflush(port,TCIFLUSH);
	    tcflush(port,TCIFLUSH);
	    timeout++;
	    usleep(100);
	    if(timeout>10){
	      fprintf(stderr,"read timed out\n");
	      char control=STOP;
	      write(port, &control, 1);
	      tcdrain(port);
	      exit(0);
	    }
	    
	  }
	  //usleep(500);
	  
	}
	
	
	break;
	default:
	  fprintf(stderr, "Usage: %s [-fdt] [file...]\n", argv[0]);
	  exit(EXIT_FAILURE);
    }
  }
  
  
}