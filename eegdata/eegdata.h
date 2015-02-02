#ifndef EEGDATA_H
#define EEGDATA_H

#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <glib/gprintf.h>
#include "cvanalyze.h"


#ifdef __cplusplus
extern "C" {
#endif

#define FRAMEDELAY 50
//#define BUFFERSIZE (SAMPLESIZE*2)
#define DEFAULT_FILE    "./telepathy-eeg.dat"
#define RXTXPROCESS     "./eegdatarxtx"
#define FIFONAME	"./eegdatafifo"
#define SAVE_FILE	"./eegdatasettings"
       

  
#define UI_FILE "ui3.ui"
#define SCOPEWIDTH 640
#define SCOPEHEIGHT 480
#define YSCALE ((float)SCOPEHEIGHT/(float)MAXSAMPLE)
#define XSCALE (SCOPEWIDTH/(SAMPLESIZE/2))

  
  
int rxpid;  
FILE* rxdaemon;
char mode[8];
  

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
void pointer_shift(unsigned short **a);
void trace(cairo_t* sc,unsigned short *data);
int SetupConnection(int port, int baudRate);
int receive(int port,unsigned char *rxbuffer,unsigned int length);
void fakepackets(unsigned int length);
void read_settings_file(void);
void save_settings_file(void);
void getdata(void);
int setrange(int range);
void writepackets(unsigned short *rxbuffer);


#ifdef __cplusplus
} 
#endif


#endif