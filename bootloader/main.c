/* 
 *
 * This is the bluetooth bootloader for the telepathy EEG project
 * The main function tests the p variable to see if it is set
 *
 * A set mark variable means a successful upload has occured
 *
 *
 * The boot code proceeds to address 0x700
 *
 *
 * If mark is not set the code jumps to the bootloader
 * to wait for a connection at 38400 to upload a program
 *
 *
 *
 * Created on May 7, 2015, 11:09 AM
 * adapted from code found at http://www.picprojects.net/serialbootloader/
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

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
//these go to the bluetooth
//outputs must be open drain because the bluetooth module is 3.3v and the microcontroller is 5v
#define ERROR_PIN PORTAbits.RA6
#define CMD_PIN PORTCbits.RC0
#define TX_PIN	PORTCbits.RC6
#define RX_PIN	PORTCbits.RC7



#define PROG_START 0x700
/*
:BBAAAATT[DDDDDDDD]CC

where
: is start of line marker
BB is number of data bytes on line
AAAA is address in bytes
TT is type. 00 means data, 01 means EOF and 04 means extended address
DD is data bytes, number depends on BB value
CC is checksum (2s-complement of number of bytes+address+data)

*/
/* ------------------------------------------------------------------------- */
/* Intel HEX file record types                                               */
/* ------------------------------------------------------------------------- */
#define HEX_DATA_REC        0   /* data record */
#define HEX_EOF_REC         1   /* end of file record */

/* ------------------------------------------------------------------------- */
/* Intel HEX file section start offsets                                      */
/* ------------------------------------------------------------------------- */
#define HEX_LEN_START       1   /* start of record length */
#define HEX_ADDR_START      3   /* start of address */
#define HEX_TYPE_START      7   /* start of record type */
#define HEX_DATA_START      9   /* start of data */
#define HEX_LINE_LEN_MAX    50  /* maximum length a line in the HEX file */

#define HEX_HEADER_LEN      4   /* lenght of lenght-, address- and type field in byte */
#define XON 0x11
#define XOFF 0x13
#define ACK                 0x06    /* positive acknowledge (ACK) */
#define NAK                 0x15    /* negative acknowledge (NAK) */

#define WORD16(msb, lsb)  (((unsigned int)msb << 8) | (lsb))



unsigned char delay_count1;
// I discovered by experimentation that if functions have the same name
// in the bootloader as the program they might be called across boundaries.
// So this function should not have the same name as the one in the program
// I would like to take advantage of this but I don't know how to link that way. 

void _DELAY__100us(unsigned int count){
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

void _TX__String(const char *stringtosend,unsigned char len){
    unsigned char i=0;
    while(i<len){
        TXREG = stringtosend[i++];
        while(!TRMT);
    }
}
void _REQ__Sequence(void){
	GIE=0;
	WREN=1;
	EECON2=0x55;
	EECON2=0xAA;
	WR=1;
	NOP();
	NOP();
	WREN=0;
	GIE=1;
}

void _EE__write(unsigned char addr,unsigned char c){
	EEADRL = addr;
	EEDATL = c;
    CFGS=0;
    EEPGD=0;
    _REQ__Sequence();
    while(WR);

}

unsigned char _EE__read(unsigned char addr)
{

	EEADR = addr;
    CFGS=0;
    EEPGD=0;
    RD = 1;
    return  EEDATA;

}
#define FLASH_SIZE 32

void _FLASH__copy(unsigned short *source_addr, unsigned short dest_addr)
{
	// variable declarations
	unsigned char index;
	
	while(WR);	// in case of any prior EEPROM writes
	EEPGD=1;	// select program memory
	CFGS=0;
	LWLO=1;

    EEADRL = (unsigned char)(dest_addr);			// LS Byte of Program Address to Erase
    EEADRH = (unsigned char)(dest_addr>>8);		// MS Byte of Program Address to Erase

    // Start row erase
    FREE = 1;				// Enable Row Erase operation	
    _REQ__Sequence();     // Order row erasure
    FREE = 0;				// Enable Row Erase operation	
    
    for(index = 0; index < FLASH_SIZE; index++)
    {
        EEDATA=source_addr[index];
        EEDATH=source_addr[index] >> 8;
       //erase the buffer
        source_addr[index]=0xFFFF;

        EEADRL=(unsigned char)(dest_addr+index);	
        EEADRH=(unsigned char)((dest_addr+index)>>8);

        if (index == FLASH_SIZE-1)
            LWLO=0;

        _REQ__Sequence();	
    }
}


/* ========================================================================= */
/* get_hexbyte                                                               */
/* returns byte representation of 2 ASCII characters pointed to by *hex      */
/* ========================================================================= */

unsigned char _GET__hexbyte(unsigned char *hex)
{
    unsigned char i;
    unsigned char ret = 0;
    unsigned char ch;

    for (i=0; i < 2; i++)
    {
        ch = *hex;

        /* convert character to integer value */
        if (ch >= 'A')
        {
            ch = ch - 'A' + 10;
        }
        else
        {
            ch = ch - '0';
        }
        ret = (ret << 4) | ch;
        hex++;
    }

    return (ret);
}
unsigned char _CHECK__checksum(unsigned char *hex, unsigned char reclen)
{
    unsigned char checksum = 0;
    unsigned char i;

    // add all byte values, incl. checksum!
    for (i=0; i <= (reclen + HEX_HEADER_LEN); i++)
    {
        checksum += _GET__hexbyte(hex);
        hex += 2;
    }

    // checksum is zero if checksum is correct
    return (checksum);
}

eeprom unsigned char mark=0;

void _BOOT__loader(void) {
 
    unsigned short addr=0;
    unsigned char reclen;
    unsigned char rectype;
    unsigned char hexend = 0;
    unsigned char idx=0;
    unsigned char blocks=0;
    unsigned char message[16];    
    unsigned char buffer[HEX_LINE_LEN_MAX];
    unsigned short block[FLASH_SIZE];

    unsigned char ch;
//cmd pin is used for hc-05 only
// hm-10 is in setup mode until it connects
/*

 SETUP SERIAL UART
 8e6/(4*(51+1)) = 38461


 */

    TXSTAbits.SYNC=0;			// Disable Synchronous/Enable Asynchronous
    RCSTAbits.SPEN=1;			// Enable serial port
    RCSTAbits.CREN=1;
    RCSTAbits.RX9=0;                    // 8 bit
    TXSTAbits.TXEN=1;			// Enable transmission mode
    SPBRG=51;                           //38400
    BAUDCTLbits.BRG16=1;
    TXSTAbits.BRGH=1;

    _DELAY__100us(10000);

/*

 SET BLUETOOTH BAUD RATE

 */
//38400bps 1 stop bit no parity
    _TX__String("AT+UART=38400,1,0\r\n",19);
    do{ //wait for ok
        while(!RCIF);
    }while(RCREG != '\n');


    CMD_PIN=0;//setup done

/*
 
 * RESET BLUETOOTH
*/


    _TX__String("AT+RESET\r\n",10);
    do{ //wait for ok
        while(!RCIF);
    }while(RCREG != '\n');

    _DELAY__100us(10000);


    ERROR_PIN=1;

    while(hexend == 0){
        idx = 0;
        /* get one line of the HEX file via RS232 until we receive LF or */
        /* we reached the end of the buffer */
        do{
             /* get one byte */
            while(!RCIF);
             ch = RCREG;
             /* save to buffer */
             buffer[idx] = ch;
             /* increment buffer index */
             idx++;
        }
        while(ch != '\n');



        /* get record length */
        reclen = _GET__hexbyte(&buffer[HEX_LEN_START]);

         if (_CHECK__checksum(&buffer[HEX_LEN_START], reclen) != 0)
        {
             _TX__String("TRANSMIT ERROR\n",15); 
             return;
        }
        else
        {

 
            /* get record type */
            rectype = _GET__hexbyte(&buffer[HEX_TYPE_START]);

            if (rectype == HEX_DATA_REC){
                    /* get address for every 32 words
                     *
                      The hex file is numbered by bytes
                     While the write operation expects 32
                     words
                     */
                    if(blocks==0){ 
                        addr = WORD16(_GET__hexbyte( &buffer[ HEX_ADDR_START ] ) , _GET__hexbyte(&buffer[ HEX_ADDR_START + 2 ] )) / 2 ;

                        //align the address to 32 words
                        idx = (unsigned char)(addr & 0x001F);  
                        if(idx > 0){
                            
                            blocks = idx;
                            addr &= ~(unsigned short)idx;
                         //   _TX__String("Fixing offset.\n",15);
                        }
                        

                    }

                    for (idx = 0; idx < reclen*2; idx+=4){
                        block[ blocks++ ] = WORD16(_GET__hexbyte(&buffer[HEX_DATA_START+idx+2]),_GET__hexbyte(&buffer[HEX_DATA_START+idx]));
                    }

/* Each record is 16 bytes but each write is 32 words don't write until you have 4 records */
                    if(blocks == FLASH_SIZE){
                /* only program code memory */
                       
                        if (addr >= PROG_START){

                            _FLASH__copy( block , addr ) ;
                            _TX__String(itoa(message , addr , 16 ),3);  
                            _TX__String("\n",1);  

                        }
                        blocks = 0;
                    }
            }
            else if (rectype == HEX_EOF_REC)
            {
            /*write what's left*/
                if(blocks>0){
                    _FLASH__copy( block , addr );
                }
                _TX__String("DONE\n",5);
                hexend = 1;
            }
        }
    }




    //set back to indicate finished
    //eeprom_write(0, 7);
    _EE__write((unsigned char)&mark,7);
    ERROR_PIN=0;
    asm("RESET");

}


void interrupt _ISR__redirect(){

    asm("goto "___mkstr(PROG_START+4));


    
}

int main(int argc, char** argv) {
// THESE ARE ASSUME TO BE HARDWIRED INTO THE CIRCUIT
   //set to 8mhz
    //OSCCONbits.SPLLEN=1;//16mhz
    OSCCONbits.SCS=00;
    OSCCONbits.IRCF=0b1110;

    ANSELCbits.ANSC0=0;
    ANSELCbits.ANSC6=0;
    ANSELCbits.ANSC7=0;

    TRISAbits.TRISA6=0;

    TRISCbits.TRISC0=0;
    TRISCbits.TRISC6=0;
    TRISCbits.TRISC7=1;

    ODCONCbits.ODCONC6=1;// Make pin open drain in order to connect to bluetooth uart at low voltage
    ODCONCbits.ODCONC0=1;
    PORTAbits.RA6=0;
    PORTCbits.RC6=0;
    CMD_PIN=1;

       unsigned char loaded = _EE__read((unsigned short)&mark);

        if(loaded!=7){
            _BOOT__loader();
        }
        asm("goto "___mkstr(PROG_START));
}


