// gcc -std=gnu99 -Wall -export-dynamic -g eegdata.c eegdata_callbacks.c kiss_fft/tools/kiss_fftr.c kiss_fft/kiss_fft.c -o eegdata  -lbiosig -lcholmod -lm -Ikiss_fft -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_objdetect `pkg-config --cflags gtk+-3.0 --libs gtk+-3.0 `
#include <stdlib.h>
#include <biosig.h>
#include <math.h>
#include <complex.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h> /* Error number definitions */

#include <termios.h> /* POSIX terminal control definitions */
#include "eegdata.h"




GtkWidget* create_window (void)
{
	GtkBuilder *builder;
	GError* error = NULL;
	char name[16];
	builder = gtk_builder_new ();
	if (!gtk_builder_add_from_file (builder, UI_FILE, &error))
	{
		g_warning ("Couldn't load builder file: %s", error->message);
		g_error_free (error);
	}
	gtk_builder_connect_signals (builder, NULL);
	GtkWidget *window= GTK_WIDGET (gtk_builder_get_object (builder, "window1"));
//	gtk_window_set_resizable (GTK_WINDOW (window), FALSE);
	video = GTK_WIDGET (gtk_builder_get_object (builder, "drawingarea1"));
	signalview = GTK_WIDGET (gtk_builder_get_object (builder, "drawingarea2"));
	devicechoice=GTK_WIDGET(gtk_builder_get_object (builder, "comboboxtext1"));
	startbutton=GTK_WIDGET(gtk_builder_get_object( builder,"togglebutton1"));
	chan1check=GTK_WIDGET(gtk_builder_get_object( builder,"checkbutton1"));
	chan2check=GTK_WIDGET(gtk_builder_get_object( builder,"checkbutton2"));
	timecheck=GTK_WIDGET(gtk_builder_get_object( builder,"checkbutton3"));
	freqcheck=GTK_WIDGET(gtk_builder_get_object( builder,"checkbutton4"));

	savedialog=GTK_WIDGET(gtk_builder_get_object( builder,"window2"));
	for(unsigned int i=0;i<10;i++){ 
	  sprintf(name,"nameentry%d",i+1);
	  savename[i]=GTK_WIDGET(gtk_builder_get_object( builder,name));
	  gtk_widget_set_name(savename[i],name);
	}
	for(unsigned int i=0;i<10;i++){
	  sprintf(name,"actionentry%d",i+1);
	  saveaction[i]=GTK_WIDGET(gtk_builder_get_object( builder,name));
	  gtk_widget_set_name(saveaction[i],name);
	}
	for(unsigned int i=0;i<10;i++){ 
	  sprintf(name,"savebutton%d",i+1);
	  savebutton[i]=GTK_WIDGET(gtk_builder_get_object( builder,name));
	  gtk_widget_set_name(savebutton[i],name);
	}
	for(unsigned int i=0;i<10;i++){ 
	  sprintf(name,"deletebutton%d",i+1);
	  deletebutton[i]=GTK_WIDGET(gtk_builder_get_object( builder,name));
	  gtk_widget_set_name(deletebutton[i],name);
	}
	for(unsigned int i=0;i<10;i++){ 
	  sprintf(name,"activecheckbutton%d",i+1);
	  activebutton[i]=GTK_WIDGET(gtk_builder_get_object( builder,name));
	  gtk_widget_set_name(activebutton[i],name);
	}
	g_object_unref (builder);
	background= cairo_image_surface_create_from_png ("./grid.png");

	return window;
}  


void microsleep(unsigned int us){
    struct timespec sleepTime;
    struct timespec returnTime;
    sleepTime.tv_sec = 0;
    sleepTime.tv_nsec = 1000;
    while (us--)
    {
      nanosleep(&sleepTime, &returnTime);
    }
}  




void pointer_shift(unsigned short **a)
{
  *(a+3)=*(a);
  for (unsigned int i = 0; i < 3; i++) {
      *(a+i) = *(a+i+1);
   }
  
}

void fakepackets(void){
  float freq1 = 5.0;
  float freq2 = 40.0;
    pointer_shift(channel1);
    pointer_shift(channel2);
    for(int i=0;i<SAMPLES;i++){
    // sine wave
      channel1[3][i]=(int)(sin(2. * M_PI * freq1 * i / SAMPLES)*ZEROSAMPLE+ZEROSAMPLE);
      channel2[3][i]=(int)(sin(2. * M_PI * freq2 * i / SAMPLES)*10+ZEROSAMPLE);
    }
}



unsigned char array_compare(unsigned short *x, unsigned short *y, unsigned short err)
{
    for (size_t i=0; i<SAMPLES; i++)
        if (fabs(x[i] - y[i]) > err)
            return FALSE;
    return TRUE;
}

/*
 * 
 * some bits of code from stackoverflow.com discussion
 */


void trace(cairo_t *cr,unsigned short data[SAMPLES],unsigned int w,unsigned int h){
  float scale =(float)h/MAXSAMPLE;
  float resolution=(float)w/SAMPLES;
  float x2=resolution;
  float y2=h;
  cairo_move_to (cr,0, (h-(data[0]*scale)));
  cairo_set_line_width(cr,2);
  for(int i=0;i<SAMPLES;i++){
      y2=h-(data[i]*scale);
      cairo_line_to (cr,x2, y2);
      x2+=resolution;
  }
}






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
    attribs.c_cc[VMIN]=1;
    attribs.c_cc[VTIME]=1;
    sleep(3);
    tcflush(port, TCIOFLUSH); 
    tcsetattr(port, TCSANOW, &attribs);
    tcgetattr(port, &attribs);
    return 1;
    
}

void createedf(void){
  
    HDRTYPE	*edfout = NULL;
    char	*filename="./telepathy-eeg.edf";


    edfout = constructHDR(3, 1);
    if (edfout == NULL) {
      perror("constructing header");
      return ;
    }
      
    edfout->TYPE=EDF;

     edfout->CHANNEL[0].PhysMin=0;	/* physical minimum */
     edfout->CHANNEL[0].PhysMax=3.3;	/* physical maximum */
     edfout->CHANNEL[0].DigMin=0;	/* digital minimum */
     edfout->CHANNEL[0].DigMax=4096;	/* digital maximum */
     edfout->CHANNEL[0].Cal=1;		/* gain factor */
     edfout->CHANNEL[0].Off=0;		/* bias */
     edfout->CHANNEL[0].OnOff=1;
     edfout->CHANNEL[0].SPR=1;
     edfout->CHANNEL[0].TOffset=1/SAMPLES;
     sprintf(edfout->CHANNEL[0].Label,"CHANNEL 1");;
 
     edfout->CHANNEL[1].PhysMin=0;	/* physical minimum */
     edfout->CHANNEL[1].PhysMax=3.3;	/* physical maximum */
     edfout->CHANNEL[1].DigMin=0;	/* digital minimum */
     edfout->CHANNEL[1].DigMax=4096;	/* digital maximum */
     edfout->CHANNEL[1].Cal=1;		/* gain factor */
     edfout->CHANNEL[1].Off=0;		/* bias */
     edfout->CHANNEL[1].OnOff=1;
     edfout->CHANNEL[1].SPR=1;
     edfout->CHANNEL[1].TOffset=1/SAMPLES;
     sprintf(edfout->CHANNEL[1].Label,"CHANNEL 2");;

     sprintf(edfout->CHANNEL[2].Label,"EDF Annotations");

    edfout->FLAG.ANONYMOUS=1;
    edfout->SPR = 1;
    edfout->SampleRate=SAMPLES;
//    edfout->EVENT.SampleRate=1;

     mkfifo(filename, 0666);    
    
     edfout = sopen(filename, "w", edfout);
     if (edfout == NULL) {
       perror("opening output file");
       return;
     }

    edfout->NRec=1;
     
//    fakepackets(edfout,SAMPLES);

    sclose(edfout);
    unlink(filename); 
  
  
  
}

void writepackets(unsigned char *rxbuffer,unsigned int length){

//this is how I get packets that are transmitted msb/lsb/msb/lsb
pointer_shift(channel1);
pointer_shift(channel2);
for(int i=0,j=0;i<length;i++,j+=4){
	channel1[3][i] = ((rxbuffer[j] & 0b00001111) << 8) |  rxbuffer[j+1];
	channel2[3][i] = ((rxbuffer[j+2] & 0b00001111) << 8 ) | rxbuffer[j+3];
//	if(!swrite(data, 1, file))perror("write failed:");
    }
    
   
}

void getdata(void){
  int		nLength;
  unsigned char	inBuf[BUFFERSIZE];
  unsigned char control[3];
  char * selected=gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(devicechoice));
  if(strcmp(selected,"simulated")==0){
      fakepackets();
  }
  else if(strcmp(selected,"bluetooth")==0){
      control[0]=CHANNEL1;
      control[1]=CHANNEL2;
      control[2]=START;
      write(port, control, 3);
      tcdrain(port);
//      microsleep(200);
      nLength = receive(port, inBuf,sizeof(inBuf));
      if(nLength)
      {
	    
	    writepackets(inBuf,nLength);
	
      }
  }  
}



int receive(int port,unsigned char *rxbuffer,unsigned int length){
  #define TIMEOUT 20
  unsigned char buffer[length];
  unsigned char timeout=TIMEOUT;
  memset(buffer,0,length);
  int chars=0;
  while(timeout){
    int result = read(port,buffer,length-chars);
    if (result >0) {
      for(int i=0;i<result;i++){
	rxbuffer[i+chars]=buffer[i];
      }
      chars+=result;
      if(chars>=length){
	return chars;
	}
    }
    else {
      microsleep(100);
      timeout--;
    }
  }
  printf("Requested %d only received %d,flushing buffer.",length,chars);
  tcflush(port,TCIFLUSH);
  tcflush(port,TCIFLUSH);
  return 0;
  
}

void allocate_buffers(void){

  // allocate an "array of arrays" of int
  channel1 = (unsigned short**)malloc( 4* sizeof(unsigned short*) ) ;
  channel2 = (unsigned short**)malloc( 4* sizeof(unsigned short*) ) ;
  spectrum1 = (unsigned short*)malloc( SAMPLES*sizeof(unsigned short) ) ;
  spectrum2 = (unsigned short*)malloc( SAMPLES*sizeof(unsigned short) ) ;

  // each entry in the array of arrays of int
  // isn't allocated yet, so allocate it
  for(unsigned int i = 0 ; i < 4 ; i++ )
  {
    // Allocate an array of int's, at each
    // entry in the "array of arrays"
    channel1[i] = (unsigned short*)malloc( SAMPLES*sizeof(unsigned short) ) ;
    channel2[i] = (unsigned short*)malloc( SAMPLES*sizeof(unsigned short) ) ;
  } 
  
  
  
}


void read_settings_file(void){
 FILE *settings=NULL;
 settings=fopen(SAVE_FILE,"r");
 if(settings==NULL)return;
 for(unsigned int i=0;i<10;i++){
    fread(&triggers[i],sizeof(TRIGGER),1,settings); 
    gtk_entry_set_text(GTK_ENTRY(savename[i]),triggers[i].name);
    gtk_entry_set_text(GTK_ENTRY(saveaction[i]),triggers[i].action);
   
  }
 fclose(settings);
}

void save_settings_file(void){
 FILE *settings=NULL;
 settings=fopen(SAVE_FILE,"w");
 if(settings==NULL)return;
 for(unsigned int i=0;i<10;i++){
      fwrite(&triggers[i],sizeof(TRIGGER),1,settings); 
 }
 fclose(settings);
}



int main( int   argc,char *argv[]){
    port=-1;
    allocate_buffers();
    GtkWidget *window;
    gtk_init(&argc, &argv);
    window = create_window ();
    read_settings_file();
    for(unsigned int i=0;i<10;i++)getpatternimage(i);
    
    gtk_widget_show (window);
    gtk_main ();

    
    
    close(port);
    return 0;

}
