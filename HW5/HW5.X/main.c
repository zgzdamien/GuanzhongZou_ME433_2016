#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"ILI9163C.h"


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
#pragma config FPLLMUL = MUL_20 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0x0001 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON// USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module


char get_bit(char c, int n)
{
    return (c >> n) & 0x01;
}
void Draw_char(char c, unsigned short x, unsigned short y)
{
    //char *asc_table[5];
    //asc_table = ASCII[c-0x20];
    char current_col;
    unsigned short i,j;
    for(i=0; i<5; i++)
    {
        current_col = ASCII[c-0x20][i];
        for(j=0;j<8;j++)
        {
            if(get_bit(current_col,j)==0x01)
            {
                LCD_drawPixel(x+i,y+j,GREEN);
            }
        }
    }
    
}

void Draw_string(char* str, unsigned short x, unsigned short y)
{
    char arr[100];
    int i=0;
    unsigned short x_start=x;
    sprintf(arr, str);
    while(arr[i])
    {
        if(x<120 && y<120)
        {
            Draw_char(arr[i],x,y);
             x+=6;
        }
       
        else if(x>=120 && y<112)
        {
            y+=10;
            x=x_start;
            Draw_char(arr[i],x,y);
        }
            
        i++;
    }
}

int main() {
 
    
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

     TRISBbits.TRISB4 = 0; //set port B4 as output pin for LED
     LATBbits.LATB4=0;
   
     SPI1_init();
     LCD_init();
     
     
     
    __builtin_enable_interrupts();
    
   LCD_clearScreen(0x0000);
   
   unsigned short x_pos;
   unsigned short y_pos;
   
   char *str="Data from IMU:";
   
   Draw_string(str,10,10);
   
    
    while(1) {
         
    }
    
    
}