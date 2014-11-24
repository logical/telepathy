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
#define BUFFERSIZE (SAMPLES*4)
#define MAXFREQUENCY (SAMPLES/4)
#define DEFAULT_COM	"/dev/rfcomm0"
#define SAVE_FILE "./eegdatasettings"
#define DEFAULT_BAUD	B1500000 
#define SAMPLERATE 512
#define DEFAULT_FILE    "telepathy-eeg.dat"


#define START 0x12
#define STOP 0x14
#define TEST 0x07;
#define XON 0x11
#define XOFF 0x13
#define CHANNEL1 '1'
#define CHANNEL2 '2'
#define UI_FILE "ui3.ui"
#define SCOPEWIDTH 640
#define SCOPEHEIGHT 480



GtkWidget *video,*signalview,*chan1check,*chan2check,*timecheck,*freqcheck,*savedialog,*startbutton,*devicechoice;
GtkWidget *savename[TRIGGERS],*saveaction[TRIGGERS],*savebutton[TRIGGERS],*deletebutton[TRIGGERS],*activebutton[TRIGGERS];
GdkPixbuf *grid;


int port;


double volts,trig,chron;
unsigned short selectedrow;


cairo_surface_t* background;
union{
    struct{
    unsigned DONE   :1;  
    unsigned DATA   :1;  
//    unsigned CH1    :1;
//    unsigned CH2    :1;
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
void trace(cairo_t *cr,unsigned short data[SAMPLES],unsigned int w,unsigned int h);

int SetupConnection(int port, int baudRate);
int receive(int port,unsigned char *rxbuffer,unsigned int length);
void fakepackets(void);
void read_settings_file(void);
void save_settings_file(void);
void getdata(void);



#ifdef __cplusplus
} 
#endif


#endif