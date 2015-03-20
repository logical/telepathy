#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <glib/gprintf.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <time.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>


#include "eegdata.h"
//#include "cvanalyze.h"


static gint timeout (gpointer data){
  //static gboolean data_ready(GIOChannel *channel, GIOCondition cond, gpointer data){
  if(rxbuffer.count>=WRITESIZE*CHANNELS){
    writepackets();
    if(!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(timecheck))){
      compare_signals();
    }
    gtk_widget_queue_draw(video);
  }
  else {
    fprintf(stderr,"Received %d bytes\n",rxbuffer.count);  
  }
  //    compare_signals();
	return controlbits.DATA;
 }



void drawingarea2_draw_event_cb(GtkWidget *widget, gpointer gdata){
  /*
   *  cairo_t* cr=gdk_cairo_create(gtk_widget_get_window(widget));
   *  
   * 
   *  cairo_set_source_rgb (cr,0, 0, 1);
   *  trace(cr,triggers[selectedrow].channel1,SCOPEWIDTH/2,SCOPEHEIGHT/2);
   *  cairo_stroke (cr);
   *  cairo_set_source_rgb (cr,1, 0, 0);
   *  trace(cr,triggers[selectedrow].channel2,SCOPEWIDTH/2,SCOPEHEIGHT/2);
   *  cairo_stroke (cr);
   */
  
}

void drawingarea1_draw_event_cb(GtkWidget *widget, gpointer gdata){
  
  //   cairo_t* cr=cairo_create(scope);
  // 
  //   cairo_set_source_surface(cr,scope,(double)(PACKET*XSCALE),0);
  //   cairo_translate(cr ,(double)-(PACKET*XSCALE),0.0);
  //   cairo_set_source_surface (cr,scope, 0, 0);
  //   cairo_paint(cr);
  //   cairo_translate(cr ,(double)(PACKET*XSCALE),0.0);
  
  
  
  
  
  
if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(timecheck))){
  cairo_surface_t* slide = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,SCOPEWIDTH,SCOPEHEIGHT);
  cairo_t* cr=cairo_create(slide);
  
  cairo_set_source_surface(cr,scope,(double)(WRITESIZE*XSCALE),0);
  cairo_translate(cr ,(double)-(WRITESIZE*XSCALE),0.0);
  cairo_set_source_surface(cr,scope,0,0);
  cairo_paint(cr);
  
  
  cairo_destroy(cr);
  
  cr=cairo_create(scope);
  
  cairo_set_source_rgba (cr, 0, 0, 0, 0);
  cairo_set_operator (cr, CAIRO_OPERATOR_SOURCE);
  cairo_paint (cr);
  cairo_set_operator (cr, CAIRO_OPERATOR_OVER);
  
  cairo_set_source_surface (cr,slide, 0, 0);
  cairo_paint(cr);

  int start=SAMPLESIZE-WRITESIZE-1;  
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan1check))){
      cairo_set_source_rgb (cr,1, 0, 0);
      trace(cr,&channel1[start],WRITESIZE);
      cairo_stroke (cr);
  }
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan2check))){
      cairo_set_source_rgb (cr,0, 0, 1);
      trace(cr,&channel2[start],WRITESIZE);
      cairo_stroke (cr);
    }
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan3check))){
      cairo_set_source_rgb (cr,0, 1, 0);
      trace(cr,&channel3[start],WRITESIZE);
      cairo_stroke (cr);
    }
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan4check))){
      cairo_set_source_rgb (cr,1, 1, 0);
      trace(cr,&channel4[start],WRITESIZE);
      cairo_stroke (cr);
    }
  cairo_surface_flush(scope);
  cairo_destroy (cr);
  
  cr=gdk_cairo_create(gtk_widget_get_window(widget));
  
  cairo_set_source_surface (cr,background, 0, 0);
  cairo_paint(cr);
  
  
  cairo_set_source_surface (cr,scope, 0, 0);
  cairo_paint(cr);
  cairo_destroy (cr);
}
else{
    cairo_t* cr=gdk_cairo_create(gtk_widget_get_window(widget));
  
  cairo_set_source_surface (cr,background, 0, 0);
  cairo_paint(cr);
  unsigned int size = (SAMPLESIZE/2)+1;
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan1check))){
      cairo_set_source_rgb (cr,1, 0, 0);
      trace(cr,spectrum1,size);
      cairo_stroke (cr);
  }
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan2check))){
      cairo_set_source_rgb (cr,0, 0, 1);
      trace(cr,spectrum2,size);
      cairo_stroke (cr);
    }
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan3check))){
      cairo_set_source_rgb (cr,0, 1, 0);
      trace(cr,spectrum3,size);
      cairo_stroke (cr);
    }
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chan4check))){
      cairo_set_source_rgb (cr,1, 1, 0);
      trace(cr,spectrum4,size);
      cairo_stroke (cr);
    }
  cairo_destroy (cr);
}
  
  
}


void devicechanged(GtkWidget *widget, gpointer gdata){
  //  this is the signal from the first radio button
  if(!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))){
    mode='d';
  }
  else{
    mode='f';
  }
  
}

void spinbutton1_value_changed_cb(GtkWidget *widget,gpointer gdata){
			int spinvalue=gtk_spin_button_get_value_as_int((GtkSpinButton*)widget);
			sprintf(command,"r %d\n",spinvalue);
}

void start( GtkWidget *widget , gpointer data){
//  sprintf(buffer,"%s %s",RXTXPROCESS,mode);
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(startbutton))){
		cb_init(&rxbuffer);
		if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(devicechoice)))sprintf(command,"f");
		else	sprintf(command,"d");

    controlbits.DATA=1;
    gtk_button_set_label((GtkButton*)startbutton,"stop");
    g_timeout_add(150,timeout,NULL);//200ms == 5fps
    
  }
  else{
		sprintf(command,"s");
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
    for(unsigned int k=0;k<4*SAMPLESIZE;k++){
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
  
  for(unsigned int k=0;k<4*SAMPLESIZE;k++){
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
  //  getdata();
  //  compare_signals();
  //  gtk_widget_queue_draw(video);
}

