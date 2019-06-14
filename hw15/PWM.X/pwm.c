#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>
#include "ili9341.h"
#include "i2c_master.h"
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ili9341.h"
#include <stdio.h>
#include <stdlib.h>

#define screen_width 240
// DEVCFG0
#pragma config DEBUG = 0 // no debugging
#pragma config JTAGEN = 0 // no jtag
#pragma config ICESEL = 0b11 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = 0 // no boot write protect
#pragma config CP = 1 // no code protect

// DEVCFG1
#pragma config FNOSC = 0b011 // use primary oscillator with pll
#pragma config FSOSCEN = 0 // turn off secondary oscillator
#pragma config IESO = 0 // no switching clocks
#pragma config POSCMOD = 0b10 // high speed crystal mode
#pragma config OSCIOFNC = 1 // disable secondary osc
#pragma config FPBDIV = 0b00 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = 0b10 // do not enable clock switch
#pragma config WDTPS = 0b10100 // use slowest wdt
#pragma config WINDIS = 1 // wdt no window mode
#pragma config FWDTEN = 0 // wdt disabled
#pragma config FWDTWINSZ = 0b11 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = 0b001 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = 0b111 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = 0b001 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = 0b001 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = 0 // USB clock on

// DEVCFG3
#pragma config USERID = 0x5877 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

    unsigned char red[screen_width];
    unsigned char blue[screen_width];
    unsigned char green[screen_width];

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
    TRISAbits.TRISA4 = 0; //make A4 a output pin
    LATAbits.LATA4 = 1;  //turn on the LED
    

    
    //Timer 2
    T2CONbits.TCKPS = 0;
    PR2 = 2399;
    TMR2 = 0;
    OC4CONbits.OCM = 0b110;
    OC4RS = 600;
    OC4R = 0;
    RPB13Rbits.RPB13R = 0b0101;
    
    //Timer 3
    T3CONbits.TCKPS = 0b100;
    PR3 = 14999;
    TMR3 = 0;
    IPC3bits.T3IP = 5;
    IPC3bits.T3IS = 0;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    
    //Turn on T2 T3 and OC1
    T2CONbits.ON = 1;
    T3CONbits.ON = 1;
    OC4CONbits.ON = 1;    
    _CP0_SET_COUNT(0);
    
    LCD_init();

    __builtin_enable_interrupts();
    
    LCD_clearScreen(ILI9341_WHITE);
    
    int msg;
    unsigned char red[screen_width];
    unsigned char blue[screen_width];
    unsigned char green[screen_width];
    
    for(msg = 0; msg < screen_width; msg++){
        red[msg] = msg;
        blue[msg] =  msg;
        green[msg] = msg;
    }
    //draw plot "red" "green" "blue"
     
    LCD_plot(90,280,ILI9341_RED,red,240);
    LCD_plot(140,280,ILI9341_BLUE,blue,240);
    LCD_plot(190,280,ILI9341_GREEN,green,240);

    while(1){
        
    }

}

void __ISR(_TIMER_3_VECTOR,IPL5SOFT)Timer3ISR(void){
    
    
    LATAINV = 0b10000; //invert LED A4
    static int count = 0;
    static int dir = 1;
    count = count + dir;
    if (count > 100) {//0.1 s
        dir = -1;
    }
    else if (count < 0){
        dir = 1;
    }
    
    OC4RS = (count * 2399)/100;
    IFS0bits.T3IF = 0;
    
  

}