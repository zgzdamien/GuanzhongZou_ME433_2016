#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"i2c_master_noint.h"

#define CS LATBbits.LATB9       // chip select pin

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0x0001 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON// USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

// initialize spi1 
void initSPI1() {
  // set up the chip select pin as an output
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
  TRISBbits.TRISB7 = 0; // select pin as output 	
  CS = 1;// output high

  // Master - SPI1, pins are: SDI1(F4), SDO1(F5), SCK1(F13).  
  // we manually control SS4 as a digital output (F12)
  // since the pic is just starting, we know that spi is off. We rely on defaults here
 
  // setup spi1
  RPB8Rbits.RPB8R = 0b0011;	// set RPB8 as SDOut
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
  

                            // send a ram set status command.
//  CS = 0;                   // enable the ram
//  spi_io(0x01);             // ram write status
//  spi_io(0x41);             // sequential mode (mode = 0b01), hold disabled (hold = 0)
//  CS = 1;                   // finish the command
}

void setVoltage(char channel, unsigned int voltage) {
		
//	int dig_vol = 0;
//	int data_sent = 0;
//	dig_vol = (2^8 * floor(voltage / 3.3));
	switch(channel) {

	case 'A' :
   		//data_sent = 0x7000 | (dig_vol<<4);
		//data_sent = 0x70 | (voltage>>4);
        CS = 0;
        spi_io(0x70 | (voltage>>4));
        spi_io(voltage<<4);
        CS = 1;
    break; 
	
	case 'B' :
   		//data_sent = 0xF000 | (dig_vol<<4);
    	//data_sent = 0xF000 | (voltage<<4);
        CS = 0;
        spi_io(0xF0 | (voltage>>4));
        spi_io(voltage<<4);
        CS = 1;
    break; 

    }
//	CS = 0;
//	spi_io(data_sent & 0xFF00>> 8);  // the most significant byte
//	spi_io(data_sent & 0x00FF);      // the least significant byte 
    
    
//	CS = 1;
  
   
}


void i2c_init(void){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
}

void i2c_expander_init(void){
    i2c_master_start();
    i2c_master_send(0x40); //OPCODE
    i2c_master_send(0x00);  // Reg ADDRESS
    i2c_master_send(0xF0); //set GP7-GP4 as inputs, GP3-GP0 as output
    i2c_master_stop();
    //set latch
   
    i2c_master_start();
    i2c_master_send(0x40);
    i2c_master_send(0x0A);
    i2c_master_send(0x00);
    i2c_master_stop();
}

void setExpander(char pin,char level)
{
    char value= level<<pin;
    i2c_master_start();
    i2c_master_send(0x40);
    i2c_master_send(0x0A);
    i2c_master_send(value);
    i2c_master_stop();
}

unsigned char getExpander()
{
    i2c_master_start();
    i2c_master_send(0x40);
    i2c_master_send(0x09);
    i2c_master_restart();
    i2c_master_send(0x41);
    unsigned char input=i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return input;
}

//7SDI 8SDO 9SS
#define CS LATBbits.LATB9       // chip select pin

int main() {
    unsigned char master_read  = 0x00; 

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    // do your TRIS and LAT commands here
   
     
     TRISAbits.TRISA4 = 1; //set portA4 as input pin for button input
     TRISBbits.TRISB4 = 0; //set port B4 as output pin for LED
    
     
     i2c_init();
     i2c_expander_init();
     
    __builtin_enable_interrupts();
    
  
   
    
    while(1) {
        master_read = getExpander();
    if(master_read>>7 ==0x01)
   {
          setExpander(0,1);
   }else{
        
        setExpander(0,0);
   }
        
    }
    
    
}