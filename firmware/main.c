/* 
 * File:   main.c
 * Author: logical
 *
 * Created on December 7, 2014, 10:37 AM
 */
/*
 *   The byte format is:
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
#include <stdlib.h>
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
#define ERROR_PIN PORTAbits.RA6
//these go to the adc and digital potentiometer
#define POT_CS_PIN PORTCbits.RC1
#define ADC_CONV_PIN PORTCbits.RC2
#define SCK_PIN PORTCbits.RC3
#define SDI_PIN PORTCbits.RC4
#define SDO_PIN PORTCbits.RC5


//these got to the multiplexer
#define MPX_A_PIN PORTBbits.RB0
#define MPX_B_PIN PORTBbits.RB1
//These outputs ground unused multiplexer inputs to prevent parasitic capacitance
#define GND_IN_0 PORTBbits.RB2
#define GND_IN_1 PORTBbits.RB3
#define GND_IN_2 PORTBbits.RB4
#define GND_IN_3 PORTBbits.RB5



//these go to the bluetooth
#define CMD_PIN PORTCbits.RC0
#define TX_PIN	PORTCbits.RC6
#define RX_PIN	PORTCbits.RC7



//#define BAUD_1382400 4
//#define BAUD_460800 3
#define BAUD_230400 2
#define BAUD_38400 1

#define PACKETSIZE 8
#define MAXSAMPLE 65535 //16 bits

struct{
    unsigned char size;
    unsigned char elems[64];
    unsigned flag :1;//set when buffer is not empty
}rxbuffer,txbuffer;


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
        else if(rx==STOP){
            controlbits.DATA = 0;
            controlbits.TEST = 0;
        }
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
//this resistor is currently on the grounded resistor of a non inverting opamp
//gain=1+rf/rg
void setresistor(unsigned char value){
    //combining the digital pot and adc on the same spi
    //the adc doesn't have a cs pin it has a conv pin
    //also both the host and the adc sdo pin should be seperated
    //by a resistor to prevent bus conflicts because the pot has
    //a multiplexed sdi/sdo
    ADC_CONV_PIN=1;
    POT_CS_PIN=0;
    SSPBUF = 0;//write command
    while(!SSPSTATbits.BF);
    SSPBUF = value;// first 8 bits
    while(!SSPSTATbits.BF);
    POT_CS_PIN=1;
    ADC_CONV_PIN=0;
    delay_100us(1);
}



#define INTERVAL 120
void feedtransmitter(){
 //   feed 8 bytes
    unsigned int counter=0; //this will count the wait time to give an acurate delay
    while(txbuffer.size>0){ //this loop transmits in reverse while couting length down
        TXREG = txbuffer.elems[--txbuffer.size];
        while(!TXSTAbits.TRMT)counter++;
    }
    while(counter<INTERVAL)counter++;
}

//this loop does not need to be optimized for speed
//apparently 256 sa/s is a good sample rate for eeg
//I'm going to try to go a little higher
//because that just seems way too slow
void senddata(void){
    unsigned int count=0;
    txbuffer.size=0;
    while(controlbits.DATA){
            txbuffer.elems[txbuffer.size++]=XOFF;//stop sync byte

            count=0;
            PORTB=0b00111000;
//            MPX_A_PIN=0;                //set multiplexer channel
//            MPX_B_PIN=0;
            delay_100us(1);//settling time
            ADC_CONV_PIN=1;             //start conversion
            delay_100us(1);//conversion time
            ADC_CONV_PIN=0;             //end conversion start transmission
            PIR1bits.SSP1IF=0;          // Clear SSP interrupt bit
             SSPBUF = 0x00;              // Write dummy data byte to the buffer to initiate transmission
            while(!SSPSTATbits.BF)count++;   //receive MSB first
//            delay_100us(1);
            txbuffer.elems[txbuffer.size++] =SSPBUF;
            PIR1bits.SSP1IF=0;          // Clear SSP interrupt bit
            SSPBUF = 0x00;              // Write dummy data byte to the buffer to initiate transmission
            while(!SSPSTATbits.BF)count++;
//            delay_100us(1);
            txbuffer.elems[txbuffer.size++]=SSPBUF;
            while(count<INTERVAL)count++;


            count=0;
            PORTB=0b00110101;
//            MPX_A_PIN=1;
//            MPX_B_PIN=0;
            delay_100us(1);//settling time
            ADC_CONV_PIN=1;
            delay_100us(1);
            ADC_CONV_PIN=0;
            PIR1bits.SSP1IF=0;          // Clear SSP interrupt bit
            SSPBUF = 0x00;              // Write dummy data byte to the buffer to initiate transmission
            while(!SSPSTATbits.BF)count++;
//            delay_100us(1);
            txbuffer.elems[txbuffer.size++]=SSPBUF;
            PIR1bits.SSP1IF=0;          // Clear SSP interrupt bit
            SSPBUF = 0x00;              // Write dummy data byte to the buffer to initiate transmission
            while(!SSPSTATbits.BF)count++;
//            delay_100us(1);
            txbuffer.elems[txbuffer.size++]=SSPBUF;
            while(count<INTERVAL)count++;

            count=0;
            PORTB=0b00101110;
//            MPX_A_PIN=0;
//            MPX_B_PIN=1;
            delay_100us(1);//settling time
            ADC_CONV_PIN=1;
            delay_100us(1);
            ADC_CONV_PIN=0;
            PIR1bits.SSP1IF=0;          // Clear SSP interrupt bit
            SSPBUF = 0x00;              // Write dummy data byte to the buffer to initiate transmission
            while(!SSPSTATbits.BF)count++;
//            delay_100us(1);
            txbuffer.elems[txbuffer.size++]=SSPBUF;
            PIR1bits.SSP1IF=0;          // Clear SSP interrupt bit
            SSPBUF = 0x00;              // Write dummy data byte to the buffer to initiate transmission
            while(!SSPSTATbits.BF)count++;
//            delay_100us(1);
            txbuffer.elems[txbuffer.size++]=SSPBUF;
            while(count<INTERVAL)count++;

            count=0;
            PORTB=0b00011111;
            MPX_A_PIN=1;
            MPX_B_PIN=1;
            delay_100us(1);//settling time
            ADC_CONV_PIN=1;
            delay_100us(1);
            ADC_CONV_PIN=0;
            PIR1bits.SSP1IF=0;          // Clear SSP interrupt bit
            SSPBUF = 0x00;              // Write dummy data byte to the buffer to initiate transmission
            while(!SSPSTATbits.BF)count++;
//            delay_100us(1);
            txbuffer.elems[txbuffer.size++]=SSPBUF;
            PIR1bits.SSP1IF=0;          // Clear SSP interrupt bit
            SSPBUF = 0x00;              // Write dummy data byte to the buffer to initiate transmission
            while(!SSPSTATbits.BF)count++;
//            delay_100us(1);
            txbuffer.elems[txbuffer.size++]=SSPBUF;
            while(count<INTERVAL)count++;

            txbuffer.elems[txbuffer.size++]=XON;//start sync byte

            feedtransmitter();

    }
}

//testmode sends 4 sawtooth waves
void test(void){
    int speed=2;
    union {
        unsigned char ch[2];
        unsigned short n;
    } channel[4];

    while(controlbits.TEST){
        txbuffer.elems[txbuffer.size++]=XOFF;//start sync byte
        txbuffer.elems[txbuffer.size++]=channel[0].ch[0];
        txbuffer.elems[txbuffer.size++]=channel[0].ch[1];;
        txbuffer.elems[txbuffer.size++]=channel[1].ch[0];
        txbuffer.elems[txbuffer.size++]=channel[1].ch[1];;
        txbuffer.elems[txbuffer.size++]=channel[2].ch[0];
        txbuffer.elems[txbuffer.size++]=channel[2].ch[1];;
        txbuffer.elems[txbuffer.size++]=channel[3].ch[0];
        txbuffer.elems[txbuffer.size++]=channel[3].ch[1];;
        txbuffer.elems[txbuffer.size++]=XON;//start sync byte

        channel[0].n+=speed;
        channel[1].n+=speed*2;
        channel[2].n+=speed*3;
        channel[3].n+=speed*4;
        if(channel[0].n >= MAXSAMPLE)channel[0].n=0;
        if(channel[1].n >= MAXSAMPLE)channel[1].n=0;
        if(channel[2].n >= MAXSAMPLE)channel[2].n=0;
        if(channel[3].n >= MAXSAMPLE)channel[3].n=0;
        delay_100us(60);

        feedtransmitter();
    }
}



void SetupADC(void){
//    this is the setup for the ltc1864 16 bit spi adc
    SSPCON1bits.SSPM=0b0000; //spi clock FOSC/4
    SSPCON1bits.SSPEN=1;



}

void SetupUART(unsigned char speed){
    RCSTAbits.SPEN=0;
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



void setHM10uart(void){
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

void processcommand(void){
    char *token;
    token=strtok(rxbuffer.elems," ");
    switch(token[0]){
        case 'r':
            token=strtok(NULL," ");
            if(token!=NULL){
                int value=strtol(token,NULL,10);
                setresistor(value);
            }
        default:
            break;

    }
    rxbuffer.size=0;
    rxbuffer.flag=0;

}

void main(void){

     //set to 8mhz
//    OSCCONbits.SPLLEN=1;//16mhz
    OSCCONbits.SCS=00;
    OSCCONbits.IRCF=0b1110;

//set all pins to digital outputs
    ANSELA = 0b00000000;
    ANSELB = 0b00000000;
    ANSELC = 0b00000000;
    TRISA = 0b00000000; // output
    TRISB = 0b00000000; // output
    TRISC = 0b10010000; // output except for RX ,SDI and SCK
    PORTA=0b00000000;
    PORTB=0b00000000;
    PORTC=0b00000000;

//cmd pin is used for hc-05 only
// hm-10 is in setup mode until it connects

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
    setHM10uart();
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
