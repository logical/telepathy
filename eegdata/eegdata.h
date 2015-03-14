#ifndef EEGDATA_H
#define EEGDATA_H

#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <glib/gprintf.h>
#include "cvanalyze.h"
#include "eegdatarxtx.h"
 
#ifdef __cplusplus
extern "C" {
#endif

#define FRAMEDELAY 50
#define DEFAULT_FILE    "./telepathy-eeg.dat"
#define RXTXPROCESS     "./eegdatarxtx"
#define FIFONAME	"./eegdatafifo"
#define SAVE_FILE	"./eegdatasettings"
       

  
#define UI_FILE "ui3.ui"
#define SCOPEWIDTH 640
#define SCOPEHEIGHT 480
#define WRITESIZE 32 //number of samples to write at once
#define YSCALE ((float)SCOPEHEIGHT/(float)MAXSAMPLE)
#define XSCALE (SCOPEWIDTH/(SAMPLESIZE/2))
#define RESISTOR 255 //this sets the resistor value that sets the final amplifier gain  

#define DEFAULT_COM	"/dev/rfcomm0"
#define DEFAULT_BAUD	B230400 

  
pthread_t rxthread;

char mode;

GtkWidget *video,*signalview,*chan1check,*chan2check,*chan3check,*chan4check,*timecheck,*freqcheck,*savedialog,*startbutton,*devicechoice,*rangespin;
GtkWidget *savename[TRIGGERS],*saveaction[TRIGGERS],*savebutton[TRIGGERS],*deletebutton[TRIGGERS],*activebutton[TRIGGERS];
GdkPixbuf *grid;




double volts,trig,chron;
unsigned short selectedrow;
unsigned char RequestedRange;

cairo_surface_t *background,*scope;

union{
    struct{
    unsigned DONE   :1;  
    unsigned DATA   :1;  
    };

}controlbits;



GtkWidget* create_window (void);
void drawingarea1_draw_event_cb(GtkWidget *widget, gpointer gdata);
void drawingarea2_draw_event_cb(GtkWidget *widget, gpointer gdata);
void open_save_patterns( GtkWidget *widget , gpointer data);
void save_pattern( GtkWidget *widget , gpointer data);
void start( GtkWidget *widget , gpointer data);
void destroy (GtkWidget *widget, gpointer data);

void microsleep(unsigned int us);
void trace(cairo_t* sc,unsigned short *data,unsigned int size);
int SetupConnection(int port, int baudRate);
int receive(int port,unsigned char *rxbuffer,unsigned int length);
void fakepackets(unsigned int length);
void read_settings_file(void);
void save_settings_file(void);
void getdata(void);
int setrange(int range);
void writepackets(void);


#ifdef __cplusplus
} 
#endif


#endif
