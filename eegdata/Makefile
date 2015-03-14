



CC= gcc

LD=ld
CFILES=eegdata.c eegdata_callbacks.c eegdatarxtx.c circular_buffer.c

CFLAGS= --std=gnu99 -rdynamic -pthread	
CLIBS=	-lm \
	-lglib-2.0 \
	-lpangocairo-1.0\
	-lpango-1.0\
	-latk-1.0\
	-lcairo-gobject\
	-lcairo\
	-lgdk_pixbuf-2.0\
	-lgio-2.0\
	-lgobject-2.0\
	-lgtk-3\
	-lgdk-3\
	-lgdk_pixbuf-2.0\
	-lglib-2.0 

CPATHS=	-pthread \
	-I/usr/include \
	-I/usr/include/opencv \
	-I/usr/include/gtk-3.0 \
	-I/usr/include/at-spi2-atk/2.0\
	-I/usr/include/at-spi-2.0\
	-I/usr/include/dbus-1.0\
	-I/usr/lib/x86_64-linux-gnu/dbus-1.0/include\
	-I/usr/include/gtk-3.0\
	-I/usr/include/gio-unix-2.0/\
	-I/usr/include/mirclient\
	-I/usr/include/mircommon\
	-I/usr/include/cairo\
	-I/usr/include/pango-1.0\
	-I/usr/include/harfbuzz\
	-I/usr/include/pango-1.0\
	-I/usr/include/atk-1.0\
	-I/usr/include/cairo\
	-I/usr/include/pixman-1\
	-I/usr/include/freetype2\
	-I/usr/include/libpng12\
	-I/usr/include/gdk-pixbuf-2.0\
	-I/usr/include/libpng12\
	-I/usr/include/glib-2.0\
	-I/usr/lib/x86_64-linux-gnu/glib-2.0/include 

CPPFILES=cvanalyze.cpp

CPPFLAGS = -O1 

CPPLIBS = -lstdc++\
	  -larmadillo\

OFILES= eegdata.o eegdatarxtx.o eegdata_callbacks.o cvanalyze.o circular_buffer.o

all:

	$(CC) -c $(CPPFLAGS) $(CPPFILES) 
	$(CC) -c $(CFLAGS) $(CFILES) $(CPATHS)
	$(CC) $(CFLAGS) $(OFILES) $(CLIBS) $(CPPLIBS) -o eegdata


clean: 
	rm -f *.o eegdata eegdatarxtx

