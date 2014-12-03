#ifndef CVANALYZE_H
#define CVANALYZE_H

#include<opencv2/highgui/highgui.hpp>

#define TRIGGERS 10
#define SAMPLES 256
#define CHANNELS 4
#define MAXSAMPLE 4096
#define ZEROSAMPLE (MAXSAMPLE/2)


#ifdef __cplusplus
extern "C" {
#endif
unsigned short *channel1,*channel2,*channel3,*channel4;
unsigned short *spectrum1,*spectrum2,*spectrum3,*spectrum4;//just for drawing
typedef struct{
  unsigned char active;
  char name[16],action[16];
  unsigned short channel1[4*SAMPLES];
  unsigned short channel2[4*SAMPLES];
  unsigned short channel3[4*SAMPLES];
  unsigned short channel4[4*SAMPLES];
} TRIGGER;

TRIGGER	triggers[TRIGGERS];

//void frequencies(unsigned short **samples,unsigned short *spectrum);
void getpatternimage(unsigned short index);
void compare_signals(void);




#ifdef __cplusplus
}
#endif

#endif

