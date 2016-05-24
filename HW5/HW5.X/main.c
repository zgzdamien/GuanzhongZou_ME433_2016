#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"ILI9163C.h"
#include"i2c_master_noint.h"

#define SLAVE_ADDR =  0b1101011
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

static signed short temperature;
static signed short x_ang_acc;
static signed short y_ang_acc;
static signed short z_ang_acc;
static signed short x_lin_acc;
static signed short y_lin_acc;
static signed short z_lin_acc;


void i2c_init(void){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
}

void i2c_imu_init(void){
    //CTRL1_XL
    i2c_master_start();
    i2c_master_send(0b11010110); //send device address 
    i2c_master_send(0x10);
    i2c_master_send(0x80); //10000000
    i2c_master_stop();
    
    //CTRL2_G
    i2c_master_start();
    i2c_master_send(0b11010110); //send device address 
    i2c_master_send(0x11); //register add
    i2c_master_send(0x80);
    i2c_master_stop();
    
    //CTRL3_C
    i2c_master_start();
    i2c_master_send(0b11010110); //send device address 
    i2c_master_send(0x12); //register add
    i2c_master_send(0x04);
    i2c_master_stop();   
    
}


void I2C_read_multiple(char address, char reg, unsigned char *data, char length)
{
    int i;
    i2c_master_start(); // make the start bit

    i2c_master_send(address<<1|0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing

    i2c_master_send(reg); // the register to read from

    i2c_master_restart(); // make the restart bit

    i2c_master_send(address<<1|1); // write the address, shifted left by 1, or'ed with a 1 to indicate reading

    for( i=0; i<length; i++)
    {
        unsigned char r = i2c_master_recv(); // save the value returned
        *data = r;
        if(i==(length-1))
            i2c_master_ack(1); // make the ack so the slave knows we got it
        else
        {
            i2c_master_ack(0); // make the ack so the slave knows we got it
            data++; 
        }
            
         
             
    }
    i2c_master_stop(); // make the stop bit
}


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
            else
            {
                LCD_drawPixel(x+i,y+j,BLACK);
            }
        }
    }
    
}

void Draw_string(char* arr, unsigned short x, unsigned short y)
{
    int i=0;
    unsigned short x_start=x;
    while(arr[i])
    {
        if(x<124 && y<120)
        {
            Draw_char(arr[i],x,y);
             x+=6;
        }
       
        else if(x>=124 && y<112)
        {
            y+=10;
            x=x_start;
            Draw_char(arr[i],x,y);
        }
            
        i++;
    }
}

int main() {
 
    unsigned char whoami;
    unsigned char value[14];
    signed short temp_val;
    signed short parameter [7];
    
    int count;
    
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
     
     i2c_init();
     i2c_imu_init();
     
     SPI1_init();
     LCD_init();
     
     
     
    __builtin_enable_interrupts();
    
   LCD_clearScreen(0x0000);
   
   unsigned short x_pos;
   unsigned short y_pos;
   char arr[100];
   char *str="Data from IMU:";
   
    sprintf(arr, str);
    Draw_string(arr,10,10);
    
    while(1) {
           
         _CP0_SET_COUNT(0);
        
        while(_CP0_GET_COUNT() < 100000) { 
            ;
       }
            I2C_read_multiple(0b1101011, 0x0F,&whoami, 1);

            if(whoami==0x69)
           {
                LATBbits.LATB4=1;
                //sprintf(arr,"%x",whoami);
                //Draw_string(arr,10,30);
           }
           while(_CP0_GET_COUNT() < 200000) { 
            ;
           }
            LATBbits.LATB4=0;
        I2C_read_multiple(0b1101011, 0x20, value, 14);
        for(count=0;count<7; count++)
        {
             temp_val = ( value[2*count+1]<< 8)| value[2*count];
             parameter[count]=temp_val; 
        }
        temperature = parameter[0];
        sprintf(arr,"Temperature: %d",-parameter[0]);
        Draw_string(arr,10,30+0);
        x_ang_acc = parameter[1];
        sprintf(arr,"X_angl: %d",parameter[1]);
        Draw_string(arr,10,30+10);
        y_ang_acc = parameter[2];
        sprintf(arr,"Y_angl: %d",parameter[2]);
        Draw_string(arr,10,30+20);
        z_ang_acc = parameter[3];
        sprintf(arr,"Z_angl: %d",parameter[3]);
        Draw_string(arr,10,30+30);
        x_lin_acc = parameter[4];
        sprintf(arr,"X_acc: %f",-parameter[4]/32768.0*20);
        Draw_string(arr,10,30+40);
        y_lin_acc = parameter[5];
        sprintf(arr,"Y_acc: %f",-parameter[5]/32768.0*20);
        Draw_string(arr,10,30+50);
        z_lin_acc = parameter[6];
        sprintf(arr,"Z_acc: %f",-parameter[6]/32768.0*20);
        Draw_string(arr,10,30+60);
     
    }
    
    
}