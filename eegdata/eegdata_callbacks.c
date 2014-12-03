#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <glib/gprintf.h>
#include <stdlib.h>
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
//#include "cvanalyze.h"

/*
static gint timeout (gpointer data)
{
    getdata();
    compare_signals();
    gtk_widget_queue_draw(video);
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(startbutton)))return TRUE;
    return FALSE;
}
*/


void drawingarea2_draw_event_cb(GtkWidget *widget, gpointer gdata){
  
  
  cairo_t* cr=gdk_cairo_create(gtk_widget_get_window(widget));
  cairo_set_source_surface (cr,background, 0, 0);
  cairo_paint(cr);

  cairo_set_source_rgb (cr,0, 0, 1);
  trace(cr,triggers[selectedrow].channel1,SCOPEWIDTH/2,SCOPEHEIGHT/2);
  cairo_stroke (cr);
  cairo_set_source_rgb (cr,1, 0, 0);
  trace(cr,triggers[selectedrow].channel2,SCOPEWIDTH/2,SCOPEHEIGHT/2);
  cairo_stroke (cr);
  
  
}

void drawingarea1_draw_event_cb(GtkWidget *widget, gpointer gdata){
  cairo_t* cr=gdk_cairo_create(gtk_widget_get_window(widget));

  
  cairo_set_source_surface (cr,background, 0, 0);
  cairo_paint(cr);
  
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan1check))){
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(timecheck))){
      cairo_set_source_rgb (cr,1, 0, 0);
      trace(cr,channel1+(SAMPLES*3),SCOPEWIDTH,SCOPEHEIGHT);
      cairo_stroke (cr);
    }
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(freqcheck))){
      cairo_set_source_rgb (cr,1, .5, .5);
      trace(cr,spectrum1,SCOPEWIDTH,SCOPEHEIGHT);
      cairo_stroke (cr);
    }
    
  }
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan2check))){
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(timecheck))){
      cairo_set_source_rgb (cr,0, 0, 1);
      trace(cr,channel2+(SAMPLES*3),SCOPEWIDTH,SCOPEHEIGHT);
      cairo_stroke (cr);
    }
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(freqcheck))){
      cairo_set_source_rgb (cr,.5, .5, 1);
      trace(cr,spectrum2,SCOPEWIDTH,SCOPEHEIGHT);
      cairo_stroke (cr);
    }
    
  }
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan3check))){
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(timecheck))){
      cairo_set_source_rgb (cr,0, 1, 0);
      trace(cr,channel3+(SAMPLES*3),SCOPEWIDTH,SCOPEHEIGHT);
      cairo_stroke (cr);
    }
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(freqcheck))){
      cairo_set_source_rgb (cr,.5, 1, .5);
      trace(cr,spectrum3,SCOPEWIDTH,SCOPEHEIGHT);
      cairo_stroke (cr);
    }
    
  }
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan4check))){
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(timecheck))){
      cairo_set_source_rgb (cr,1, 1, 0);
      trace(cr,channel4+(SAMPLES*3),SCOPEWIDTH,SCOPEHEIGHT);
      cairo_stroke (cr);
    }
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(freqcheck))){
      cairo_set_source_rgb (cr,1, 1, .5);
      trace(cr,spectrum4,SCOPEWIDTH,SCOPEHEIGHT);
      cairo_stroke (cr);
    }
    
  }
  
  cairo_destroy (cr);
  printf("FRAME\n");
  
}


void devicechanged(GtkWidget *widget, gpointer gdata){
//  this is the signal from the first radio button
  if(!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))){
     if(port!=-1){close(port);}
      port = open(DEFAULT_COM, O_RDWR | O_NOCTTY |O_NONBLOCK);
      if (port == -1){
	  perror("Opening COM port");
	  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget),FALSE);
	  return;
      }
      if (!SetupConnection(port, DEFAULT_BAUD)) {
	  perror("SetupConnection");
	  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget),FALSE);
	  return;
      }
      
  }
  else{
      if(port!=-1){
	close(port);
      }
  }
}

void start( GtkWidget *widget , gpointer data){
  
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(startbutton))){
//    g_timeout_add(100,timeout,NULL);
    controlbits.DATA=1;
    gtk_button_set_label((GtkButton*)startbutton,"stop");
    }
    else{
    controlbits.DATA=0;
    gtk_button_set_label((GtkButton*)startbutton,"start");
  }
}


void rowselect( GtkWidget *widget , gpointer data){
  GdkRGBA red;
  red.red=1.0;
  red.blue=0;
  red.green=0;
  red.alpha=1.0;
 char *name=(char*)gtk_widget_get_name(widget); 
  
 name=strtok(name,"nameentry");
 int index=atoi(name)-1;
  for(unsigned int i=0;i<10;i++){
    gtk_widget_override_background_color(savename[i],0,NULL);
    gtk_widget_override_background_color(saveaction[i],0,NULL);
  }
  gtk_widget_override_background_color(savename[index],0,&red);
  gtk_widget_override_background_color(saveaction[index],0,&red);
  selectedrow=index;
  gtk_widget_queue_draw(signalview); 
  
}


void activaterow( GtkWidget *widget , gpointer data){
 char *name=(char*)gtk_widget_get_name(widget); 
  
 name=strtok(name,"activecheckbutton");
 int index=atoi(name)-1;
 printf("%s = %d\n",name,index);
 triggers[index].active=gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(activebutton[index]));
 
}

void saveentry( GtkWidget *widget , gpointer data){
  
 char *widgetname=(char*)gtk_widget_get_name(widget); 
  
 widgetname=strtok(widgetname,"savebutton");
 int index=atoi(widgetname)-1;

 char* name = (char*)gtk_entry_get_text(GTK_ENTRY(savename[index]));
 char* action = (char*)gtk_entry_get_text(GTK_ENTRY(saveaction[index]));
//save if the name and action are present
  if((strcmp(name,"")!=0) && (strcmp(action,"")!=0)){
    strcpy(triggers[index].name,name);
    strcpy(triggers[index].action,action);
    for(unsigned int k=0;k<4*SAMPLES;k++){
      triggers[index].channel1[k]=channel1[k];
      triggers[index].channel2[k]=channel2[k];
    }
  }
  selectedrow=index;
  getpatternimage(index);
  gtk_widget_queue_draw(signalview);
  save_settings_file();
}

void deleteentry( GtkWidget *widget , gpointer data){
  char *widgetname=(char*)gtk_widget_get_name(widget); 

  widgetname=strtok(widgetname,"deletebutton");
  int index=atoi(widgetname)-1;

//save if the name and action are present
  strcpy(triggers[index].name,"");
  strcpy(triggers[index].action,"");
  gtk_entry_set_text(GTK_ENTRY(savename[index]),"");
  gtk_entry_set_text(GTK_ENTRY(saveaction[index]),"");

  for(unsigned int k=0;k<4*SAMPLES;k++){
      triggers[index].channel1[k]=0;
      triggers[index].channel2[k]=0;
      triggers[index].channel3[k]=0;
      triggers[index].channel4[k]=0;
  }
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(activebutton[index]),FALSE);
  triggers[index].active=0;
  save_settings_file();
}


void open_save_patterns( GtkWidget *widget , gpointer data){
//  this will stop the program from updating and preserve the reading
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(startbutton),FALSE);
//  find the emplty row and change the color to red
  selectedrow=0;
  
  for(unsigned int i=0;i<10;i++){
    
    gtk_entry_set_text(GTK_ENTRY(savename[i]),triggers[i].name);
    gtk_entry_set_text(GTK_ENTRY(saveaction[i]),triggers[i].action);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(activebutton[i]),triggers[i].active);
  }
  gtk_widget_show(savedialog);
  gtk_widget_queue_draw(signalview);
}

void close_save( GtkWidget *widget , gpointer data){
  gtk_widget_hide(savedialog);
  
}



void destroy (GtkWidget *widget, gpointer data)
{
  gtk_main_quit ();
  controlbits.DONE=1;  
}

void test (GtkWidget *widget, gpointer data)
{
    getdata();
    compare_signals();
    gtk_widget_queue_draw(video);
}

