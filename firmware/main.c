/*
 * File:   hc-05.c
 * Author: logical
 *
 * Created on June 26, 2014, 6:53 AM
 */
/*
The byte format is:
 * 
 * MSB channel 1/LSB channel 1/MSB channel 2/LSB channel 2/MSB channel 3/LSB channel 3/MSB channel 4/LSB channel 4/
 * the control character are:
 * 
 * s sample 256 samples and send 512 bytes per channel
 *
 * t to test transmitter by sending "hello\n"
 *
 * R+ to increase ADC range and R- to decrease ADC range
 *
 *
 */
//#include <stdlib.h>
#include <string.h>
#include <pic16f1788.h>

#define NO_BIT_DEFINES


// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON      // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low Power Brown-Out Reset Enable Bit (Low power brown-out is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)


/*pin defines*/
#define CMD_PIN PORTCbits.RC1
#define ERROR_PIN PORTCbits.RC2 //for error detection

#define TX_PORT	6
#define RX_PORT	7
#define TX_BIT	(1<<TX_PORT)
#define RX_BIT	(1<<RX_PORT)


//#define BAUD_1382400 4
//#define BAUD_460800 3
#define BAUD_230400 2
#define BAUD_38400 1

#define PACKET_SIZE 8


struct{
    unsigned char size;
    unsigned char elems[64];
    unsigned flag :1;
}rxbuffer;

#define ADC_CHANNEL_1 0b00000//pin 2
#define ADC_CHANNEL_2 0b00001//pin 3
#define ADC_CHANNEL_3 0b01000//pin 23
#define ADC_CHANNEL_4 0b01001//pin 24


#define XON 0x11
#define XOFF 0x13
#define START 0x12
#define STOP 0x14
#define TESTMODE 0x07



union{
    struct{
    unsigned FLOW    :1;
    unsigned DATA    :1; //call data function
    unsigned TEST    :1;//call test function
    unsigned RX      :1;
    unsigned TX      :1;
    unsigned CH1     :1;
    unsigned CH2     :1;
    };

}controlbits;


unsigned char delay_count1;


void delay_100us(unsigned int count){
/*
; Delay = 0.0001 seconds
; Clock frequency = 8 MHz

; Actual delay = 0.0001 seconds = 200 cycles
; Error = 0 %
*/
//;798 cycles
    while(count--){
        //			;196 cycles
        asm("movlw	0x41");
        asm("movwf	_delay_count1");
        asm("Delay_1    ");
        asm("decfsz	_delay_count1, f");
        asm("goto	Delay_1");
    }

}

void flasherror(unsigned char flashes){
    for(unsigned char f=0;f<flashes;f++){
            ERROR_PIN=1;
            delay_100us(2000);
            ERROR_PIN=0;
            delay_100us(2000);
        }
            delay_100us(5000);
}
unsigned int bytecount=0;

void interrupt isr(){

    if(PIR1bits.RCIF==1) {
        unsigned char rx=RCREG;
        if(rx==START)
            controlbits.DATA = 1;
        else if(rx==XON)
            controlbits.FLOW = 1;
        else if(rx==STOP)
            controlbits.DATA = 0;
        else if(rx==TESTMODE)
            controlbits.TEST=1;
        else {
            rxbuffer.elems[rxbuffer.size] = rx;

            if(rxbuffer.elems[rxbuffer.size]==10){//linefeed
                rxbuffer.elems[rxbuffer.size+1]=0;
                rxbuffer.flag=1;
            }
            rxbuffer.size++;
        }
        PIR1bits.RCIF=0;
    }
}



//this loop does not need to be optimized for speed
//apparently 256 sa/s is a good sample rate for eeg
//I'm going to try to go a little higher
//because that just seems way too slow
void senddata(void){
    for(unsigned int i=0;i<PACKET_SIZE;i++){

            ADCON0bits.CHS=ADC_CHANNEL_1;
            delay_100us(2);
            if(i>0)TXREG=ADRESL;
            ADCON0bits.GO = 1; //Start conversion
            delay_100us(2);
            TXREG=ADRESH;

            ADCON0bits.CHS=ADC_CHANNEL_2;
            delay_100us(2);
            TXREG=ADRESL;
            ADCON0bits.GO = 1; //Start conversion
            delay_100us(2);
            TXREG=ADRESH;

            ADCON0bits.CHS=ADC_CHANNEL_3;
            delay_100us(2);
            TXREG=ADRESL;
            ADCON0bits.GO = 1; //Start conversion
            delay_100us(2);
            TXREG=ADRESH;

            ADCON0bits.CHS=ADC_CHANNEL_4;
            delay_100us(2);
            TXREG=ADRESL;
            ADCON0bits.GO = 1; //Start conversion
            delay_100us(2);
            TXREG=ADRESH;
        
    }
    TXREG=ADRESL;
    while(!TXSTAbits.TRMT);
    controlbits.DATA=0;

}

void SetupADC(void){
// MINIMUM voltage for the ADC is 1.8v
//after some tinkering I have found the best configuration for the ADC
//is to set the FVR to 2v and attach it to ref+


//4 channels should be enough.
    TRISB = 0b00011000;
    ANSELB = 0b00011000; //RB2=AN8 RB3=AN9

    TRISA = 0b11100111;
    ANSELA = 0b00011111; //RA1=AN1 RA2=AN2,RA4=DAC4 RA5=DAC2


    FVRCONbits.FVREN=1;
    FVRCONbits.ADFVR=0b10; //set to 2.048

    ADCON1bits.ADCS =0b111;
    ADCON1bits.ADFM = 1;//2's complement
    ADCON0bits.ADRMD = 0;
    ADCON1bits.ADNREF = 0;//set to vss
    ADCON1bits.ADPREF = 0b11;//set to FVR
    ADCON2bits.CHSN=0b1111;
    ADCON0bits.ADON = 1;

//channel select for differential
    //    ADCON2bits.CHSN=;
}

void SetupUART(unsigned char speed){
    RCSTAbits.SPEN=0;
/*
//    1382400 does not have a match in termios
//    the closest rate is 1500000
    if(speed==BAUD_1382400){//32e6/(4*(5+1))=1333333
        SPBRG=5;
        BAUDCTLbits.BRG16=1;
	TXSTAbits.BRGH=1;
    }
    else if(speed==BAUD_460800){//32e6/(4*(17+1))=444444
        SPBRG=17;
        BAUDCTLbits.BRG16=1;
	TXSTAbits.BRGH=1;
    }
*/
    if(speed==BAUD_230400){//8e6/(4*(8+1)) = 222222
        SPBRG=8;
        BAUDCTLbits.BRG16=1;
	TXSTAbits.BRGH=1;
    }
    else if(speed==BAUD_38400){//8e6/(4*(51+1)) = 38461
	SPBRG=51;
        BAUDCTLbits.BRG16=1;
	TXSTAbits.BRGH=1;
    }
    TXSTAbits.SYNC=0;			// Disable Synchronous/Enable Asynchronous
    RCSTAbits.SPEN=1;			// Enable serial port
    RCSTAbits.CREN=1;
    RCSTAbits.RX9=0;                    // 8 bit
    //set the receive interrupt
    PIE1bits.RCIE=1;
    INTCONbits.PEIE=1;

}




void TXstring(const char *stringtosend){
    unsigned char i=0;
    controlbits.FLOW=1;
    while(i<strlen(stringtosend)){
        if(controlbits.FLOW ){
            TXREG = stringtosend[i++];
            while(!TXSTAbits.TRMT);
        }
    }
}



void setHC05uart(void){
    SetupUART(BAUD_38400); //Initialize UART Module
    delay_100us(10);
    TXstring("AT+UART?\r\n");
    unsigned char response[32];
    ERROR_PIN=1;
    //wait for a response
    while(!rxbuffer.flag){
        //delay_100us(1);
    }
    rxbuffer.flag=0;
    while(!rxbuffer.flag){}
    ERROR_PIN=0;

    strcpy(response,rxbuffer.elems);
    rxbuffer.size=0;
    rxbuffer.flag=0;

    if(strcmp(response,"+UART:230400,1,0\r\nOK\r\n")!=0){
        TXstring("AT+UART=230400,1,0\r\n");
        ERROR_PIN=1;
        while(!rxbuffer.flag){}
        ERROR_PIN=0;

        strcpy(response,rxbuffer.elems);
        if(strcmp(response,"OK\r\n")!=0)while(1)flasherror(5);
        rxbuffer.size=0;
        rxbuffer.flag=0;

    }
    //    TXstring("AT+NAME=TELEPATHY");
}

void resetHC05(){
    TXstring("AT+RESET\r\n");
    unsigned char response[32];
    //wait for a response
    ERROR_PIN=1;
    while(!rxbuffer.flag){}
    ERROR_PIN=0;
    strcpy(response,rxbuffer.elems);
    if(strcmp(response,"OK\r\n")!=0)while(1)flasherror(2);
    rxbuffer.size=0;
    rxbuffer.flag=0;
    //startup time

}
void test(){
    TXstring("hello\n");
    controlbits.TEST=0;
}

void processcommand(void){


    rxbuffer.size=0;
    rxbuffer.flag=0;

}

void main(void){

     //set to 8mhz
//    OSCCONbits.SPLLEN=1;
    OSCCONbits.SCS=00;
    OSCCONbits.IRCF=0b1110;

    ANSELC = 0;
    TRISC = 0b10000000; // output
    CMD_PIN=1;

    SetupADC(); //Initialize ADC Module

    TXSTAbits.SYNC=0;			// Disable Synchronous/Enable Asynchronous
    RCSTAbits.SPEN=1;			// Enable serial port
    RCSTAbits.CREN=1;
    RCSTAbits.RX9=0;                    // 8 bit
    TXSTAbits.TXEN=1;			// Enable transmission mode
    //set the receive interrupt
    PIE1bits.RCIE=1;
    INTCONbits.PEIE=1;
    INTCONbits.GIE=1;

    rxbuffer.size=0;
    rxbuffer.flag=0;
    //startup time
    delay_100us(50000);
    setHC05uart();
    //pullup resistor  on CMD_PIN will keep module in command mode until ready
    CMD_PIN=0;
    resetHC05();
    delay_100us(10000);


    SetupUART(BAUD_230400);

    delay_100us(100);
    controlbits.CH1=0;
    controlbits.CH2=0;
    while(1){
        if(controlbits.DATA)senddata();
        if(controlbits.TEST)test();
        if(rxbuffer.flag)processcommand();
    }


}
