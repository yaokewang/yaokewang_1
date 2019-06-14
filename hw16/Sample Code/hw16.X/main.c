#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <stdio.h>

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
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = ON // allow multiple reconfigurations
#pragma config IOL1WAY = ON // allow multiple reconfigurations
#pragma config FUSBIDIO = OFF // USB pins controlled by USB module
#pragma config FVBUSONIO = OFF // USB BUSON controlled by USB module


#include "ili9341.h"
#include "i2c_master_noint.h"
#include "ov7670.h"

#define DIR1 LATAbits.LATA10
#define DIR2 LATAbits.LATA7
#define USER PORTBbits.RB4



void startup() {
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // pin definitions
    ANSELA = 0;
    ANSELB = 0;
    TRISAbits.TRISA7 = 0; // DIR2
    DIR2 = 1;
    TRISAbits.TRISA10 = 0; // DIR1
    DIR1 = 1;
    TRISBbits.TRISB4 = 1; // USER
    
    // OC1 is B15, goes with DIR1
    RPB15Rbits.RPB15R = 0b0101;
    
    // OC4 is A4, goes with DIR2
    RPA4Rbits.RPA4R = 0b0101;
    
    // use Timer 3 for PWM
    T3CONbits.TCKPS = 0; // Timer prescaler N=1 (1:1)
    PR3 = 2399; // PR = PBCLK / N / desiredF - 1
    TMR3 = 0; // initial TMR count is 0
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC4CONbits.OCM = 0b110;
    OC1CONbits.OCTSEL = 1; // User Timer3
    OC4CONbits.OCTSEL = 1;
    OC1RS = 0; // duty cycle
    OC1R = 0; // initialize before turning OC1 on; afterward it is read-only
    OC4RS = 0;
    OC4R = 0;
    T3CONbits.ON = 1; // turn on Timer
    OC1CONbits.ON = 1; // turn on OC1
    OC4CONbits.ON = 1;
    
    // LCD uses SPI1: A0 is SDO, A1 is SDI, B5 is CST, B14 is SCK1, A9 is DC, B7 is CS
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_DARKGREEN);
    
    // Camera uses C0-7, C8 as OC2, A8 as INT3/PCLK, B13 as INT2/HREF, C9 as INT1/VSYNC, and I2C1
    i2c_master_setup();
    ov7670_setup();
    
    // B3 is available as SCL2, B2 is available as SDA2
}

int main() {

    __builtin_disable_interrupts();

    startup();

    __builtin_enable_interrupts();
    
    int I = 0;
    char message[100];
    unsigned char d[2000]; // this is the raw camera data, both brightness and color
    unsigned char bright[1000]; // this is for just the brightness data
    
    while(1) {

        I++;
        sprintf(message,"I = %d   ", I);
        drawString(140,82,message);
        
        // horizontal read
        /*
        int c = ov7670_count_horz(d);
        sprintf(message, "c = %d   ",c);
        drawString(140,92,message); // there are 290 rows
        
        int x = 0, x2 = 1;
        int y = 0;
        int dim = 0;
        for(x = 0; x < c/2; x++, x2=x2+2){
            dim = d[x2]>>3;
            bright[x] = d[x2];
            for(y=0;y<32;y++){
                if (y == dim){
                    LCD_drawPixel(y+30,x,ILI9341_BLACK);
                }
                else {
                    LCD_drawPixel(y+30,x,ILI9341_WHITE);
                }
            }
        }
        */
        
        // vertical read
        int c = ov7670_count_vert(d);
        sprintf(message, "c = %d   ",c);
        drawString(140,92,message);
        
        int x = 0, x2 = 0;
        int y = 0;
        int dim = 0;
        for(x = 0; x < c/2; x++, x2=x2+2){
            dim = d[x2]>>3;
            bright[x] = d[x2];
            for(y=0;y<32;y++){
                if (y == dim){
                    LCD_drawPixel(x,y+30,ILI9341_BLACK);
                }
                else {
                    LCD_drawPixel(x,y+30,ILI9341_WHITE);
                }
            }
        }
        
        // at this point, bright has c/2 values in it, each value is a brightness of a pixel
        // loop through bright and calculate where the middle of the dip or bump is
        // then set the motor speed and direction to follow the line
        int i = 0;
        int sum = 0;
        int sumR = 0;
        int com = 0;
        int avg = 0;
        // find the average brightness
        for (i=0;i<c/2;i++){
            sum = sum + bright[i];
        }
        avg = sum / c/2;
        // threshold and center of mass calculation
        sum = 0;
        for (i=0;i<c/2;i++){
            if (bright[i]<avg){
                // count this pixel
                LCD_drawPixel(i,30,ILI9341_BLUE); // visualize which pixels we're counting
                sum = sum + 255;
                sumR = sumR + 255*i;
            }
            else {
                LCD_drawPixel(i,30,ILI9341_WHITE);
                // don't count this pixel
            }
        }
        // only use com if the camera saw some data
        if (sum>10){
            com = sumR/sum;
        }
        else {
            com = c/2/2;
        }
        // draw the center of mass as a bar
        for(y=0;y<32;y++){
            LCD_drawPixel(com,y+30,ILI9341_RED);
        }
        int speed = 0;
        int e = 0;
        // try to keep com at c/2/2 using the motors
        DIR1 = 1; // depending on your motor directions these might be different
        DIR2 = 1;
        // if com is less than c/2/2, then slow down the left motor, full speed right motor
        // if com is more than c/2/2, then slow down right motor, full speed left motor
        // things to play with: the slope of the line, the value that determines when the motor is not full speed
        if (com < c/2/2){
            e = c/2/2 - com;
            speed = 2399 - (2399/c/2/2)*e; // when the com is all the way over, the motor is all off
            if(speed > 2399){
                speed = 2399;
            }
            if(speed < 0){
                speed = 0;
            }
            OC1RS = 2399;
            OC4RS = speed;
        }
        else {
            e = com - c/2/2;
            speed = 2399 - (2399/c/2/2)*e; // when the com is all the way over, the motor is all off
            if(speed > 2399){
                speed = 2399;
            }
            if(speed < 0){
                speed = 0;
            }
            OC1RS = speed;
            OC4RS = 2399;
        }

    }
}


  void i2c_master_setup(void) {
  I2C1BRG = 230;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
                                    // look up PGD for your PIC32
  I2C1CONbits.ON = 1;               // turn on the I2C1 module
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C1CONbits.SEN = 1;            // send the start bit
    while(I2C1CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C1CONbits.RSEN = 1;           // send a restart 
    while(I2C1CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C1TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C1STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C1STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
    // ("I2C2 Master: failed to receive ACK\r\n");
      //while(1){}
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C1CONbits.RCEN = 1;             // start receiving data
    while(!I2C1STATbits.RBF) { ; }    // wait to receive the data
    return I2C1RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C1CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C1CONbits.ACKEN = 1;            // send ACKDT
    while(I2C1CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C1CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C1CONbits.PEN) { ; }        // wait for STOP to complete
}


void LCD_init() {
    int time = 0;
    
    CS = 0; // CS
   
    LCD_command(ILI9341_SWRESET);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 7200000) {} // 300ms

    LCD_command(0xEF);
  	LCD_data(0x03);
	LCD_data(0x80);
	LCD_data(0x02);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xCF);
  	LCD_data(0x00);
	LCD_data(0xC1);
	LCD_data(0x30);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xED);
  	LCD_data(0x64);
	LCD_data(0x03);
	LCD_data(0x12);
    LCD_data(0x81);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xE8);
  	LCD_data(0x85);
	LCD_data(0x00);
	LCD_data(0x78);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xCB);
  	LCD_data(0x39);
	LCD_data(0x2C);
	LCD_data(0x00);
    LCD_data(0x34);
    LCD_data(0x02);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xF7);
  	LCD_data(0x20);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xEA);
  	LCD_data(0x00);
	LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_PWCTR1);
  	LCD_data(0x23);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_PWCTR2);
  	LCD_data(0x10);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_VMCTR1 );
  	LCD_data(0x3e);
    LCD_data(0x28);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_VMCTR2);
  	LCD_data(0x86);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_MADCTL);
  	LCD_data(0x48);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
/*    
    LCD_command(ILI9341_VSCRSADD);
  	LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
 */   
    LCD_command(ILI9341_PIXFMT);
  	LCD_data(0x55);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_FRMCTR1);
  	LCD_data(0x00);
    LCD_data(0x18);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command( ILI9341_DFUNCTR);
  	LCD_data(0x08);
    LCD_data(0x82);
    LCD_data(0x27);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xF2);
  	LCD_data(0); // 1
    LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GAMMASET);
  	LCD_data(0x01);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GMCTRP1);
  	LCD_data(0x0F);
    LCD_data(0x31);
    LCD_data(0x2B);
    LCD_data(0x0C);
    LCD_data(0x0E);
    LCD_data(0x08);
    LCD_data(0x4E);
    LCD_data(0xF1);
    LCD_data(0x37);
    LCD_data(0x07);
    LCD_data(0x10);
    LCD_data(0x03);
    LCD_data(0x0E);
    LCD_data(0x09);
    LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GMCTRN1);
  	LCD_data(0x00);
    LCD_data(0x0E);
    LCD_data(0x14);
    LCD_data(0x03);
    LCD_data(0x11);
    LCD_data(0x07);
    LCD_data(0x31);
    LCD_data(0xC1);
    LCD_data(0x48);
    LCD_data(0x08);
    LCD_data(0x0F);
    LCD_data(0x0C);
    LCD_data(0x31);
    LCD_data(0x36);
    LCD_data(0x0F);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xB1);
  	LCD_data(0x00);
    LCD_data(0x10);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_SLPOUT);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_DISPON);
    
    CS = 1; // CS
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    
    CS = 0; // CS
    
    LCD_command(ILI9341_MADCTL);
    LCD_data(MADCTL_MX | MADCTL_BGR); // rotation
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    CS = 1; // CS
}

void SPI1_init() {
  //SDI1Rbits.SDI1R = 0b0000; // A0 is supposed to be SDI1 but that's not an option, oops
  RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
  TRISBbits.TRISB7 = 0; // CS is B7
  CS = 1; // CS starts high
  TRISBbits.TRISB5 = 0; // CST is B5
  CST = 1; // CS starts high

  // DC pin
  TRISAbits.TRISA9 = 0;
  DC = 1;
  
  SPI1CON = 0; // turn off the spi module and reset it
  SPI1BUF; // clear the rx buffer by reading from it
  SPI1BRG = 3; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0; // clear the overflow bit
  SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1; // master operation
  SPI1CONbits.ON = 1; // turn on spi1
}

unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void LCD_command(unsigned char com) {
    DC = 0; // DC
    spi_io(com);
    DC = 1; // DC
}

void LCD_data(unsigned char dat) {
    spi_io(dat);
}

void LCD_data16(unsigned short dat) {
    spi_io(dat>>8);
    spi_io(dat);
}

void LCD_setAddr(unsigned short x, unsigned short y, unsigned short w, unsigned short h) {
    LCD_command(ILI9341_CASET); // Column
    LCD_data16(x);
	LCD_data16(x+w-1);

	LCD_command(ILI9341_PASET); // Page
	LCD_data16(y);
	LCD_data16(y+h-1);

	LCD_command(ILI9341_RAMWR); // Into RAM
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
  // check boundary
    
    CS = 0; // CS
    
    LCD_setAddr(x,y,1,1);
    LCD_data16(color);
    
    CS = 1; // CS
}

void LCD_clearScreen(unsigned short color) {
    int i;
    
    CS = 0; // CS
    
    LCD_setAddr(0,0,ILI9341_TFTWIDTH,ILI9341_TFTHEIGHT);
	for (i = 0;i < ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT; i++){
		LCD_data16(color);
	}
    
    CS = 1; // CS
}

void XPT2046_read(int *z, unsigned short *x, unsigned short *y){
    unsigned short z1, z2;
    
    unsigned char r1, r2;
    unsigned short t1, t2;
    
    CST = 0;
    spi_io(0b10110001); // Z1
    r1 = spi_io(0x00);
    r2 = spi_io(0x00);
    CST = 1;
    z1 = ((r1<<8)|r2)>>3;
    
    CST = 0;
    spi_io(0b11000001); // Z2
    r1 = spi_io(0x00);
    r2 = spi_io(0x00);
    CST = 1;
    z2 = ((r1<<8)|r2)>>3;
    
    CST = 0;
    spi_io(0b10010001); // Y
    r1 = spi_io(0x00);
    r2 = spi_io(0x00);
    CST = 1;
    t1 = ((r1<<8)|r2)>>3;
    
    CST = 0;
    spi_io(0b11010001); // X
    r1 = spi_io(0x00);
    r2 = spi_io(0x00);
    CST = 1;
    t2 = ((r1<<8)|r2)>>3;
    
    
    *z = z1 - z2 + 4095;
    *y = t1;
    *x = t2;
    
}

void drawChar(unsigned short x,unsigned short y, char c){
    int i = 0;
    int j = 0;
    for(i=0;i<5;i++){
        for(j=0;j<8;j++){
            if((ASCII[c-0x20][i]>>j)&1==1){
                LCD_drawPixel(x+i,y+j,ILI9341_WHITE);
            }
            else{
                LCD_drawPixel(x+i,y+j,ILI9341_BLACK);
            }
        }
    }
}

void drawString(unsigned short x,unsigned short y, char * a){
    int i=0;
    while(a[i]){
        drawChar(x+i*5,y,a[i]);
        i++;
    }
}



void ov7670_setup(){
    // set C8 to OC2 at 12MHz using Timer2
    RPC8Rbits.RPC8R = 0b0101; // C8 is OC2
    T2CONbits.TCKPS = 0; // Timer2 prescaler N=1 (1:1)
    PR2 = 3; // PR = PBCLK / N / desiredF - 1
    TMR2 = 0; // initial TMR2 count is 0
    OC2CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC2RS = 1; // duty cycle = 50%
    OC2R = 1; // initialize before turning on; afterward it is read-only
    T2CONbits.ON = 1; // turn on Timer2
    OC2CONbits.ON = 1; // turn on OC1
    
    // set camera to QCIF format
    unsigned char val = 0;
    /*
    // set 0x0C to 0b00001000
    i2c_master_start();
    i2c_master_send(OV7670_SCCB_ADDRESS<<1);
    i2c_master_send(0x0C);
    i2c_master_send(0b00001000); // enable scaling
    i2c_master_stop();
    
    // set 0x12 to 0b1000
    i2c_master_start();
    i2c_master_send(OV7670_SCCB_ADDRESS<<1);
    i2c_master_send(0x12);
    i2c_master_send(0b0001000); // //set to qcif 176x144
    i2c_master_stop();
    
    
    //i2c_master_start();
    //i2c_master_send(OV7670_SCCB_ADDRESS<<1);
    //i2c_master_send(0x3E);
    //i2c_master_send(0b00011100); 
    //i2c_master_stop();
     */
    
    writeCameraRegister(0x12, 0x80); // reset all camera registers to default value
 _CP0_SET_COUNT(0);
 while(_CP0_GET_COUNT()<48000000/2){}
  writeCameraRegister(0x3A, 0x04);
  writeCameraRegister(0x12, 0x08); // was 00
  writeCameraRegister(0x17, 0x13);
  writeCameraRegister(0x18, 0x01);
  writeCameraRegister(0x32, 0xB6);
  writeCameraRegister(0x19, 0x02);
  writeCameraRegister(0x1A, 0x7A);
  writeCameraRegister(0x03, 0x0A);
  writeCameraRegister(0x0C, 0x00);
  writeCameraRegister(0x3E, 0x00);
  writeCameraRegister(0x70, 0x3A);
  writeCameraRegister(0x71, 0x35);
  writeCameraRegister(0x72, 0x11);
  writeCameraRegister(0x73, 0xF0);
  writeCameraRegister(0xA2, 0x01);
  writeCameraRegister(0x15, 0x00);
  writeCameraRegister(0x7A, 0x20);
  writeCameraRegister(0x7B, 0x10);
  writeCameraRegister(0x7C, 0x1E);
  writeCameraRegister(0x7D, 0x35);
  writeCameraRegister(0x7E, 0x5A);
  writeCameraRegister(0x7F, 0x69);
  writeCameraRegister(0x80, 0x76);
  writeCameraRegister(0x81, 0x80);
  writeCameraRegister(0x82, 0x88);
  writeCameraRegister(0x83, 0x8F);
  writeCameraRegister(0x84, 0x96);
  writeCameraRegister(0x85, 0xA3);
  writeCameraRegister(0x86, 0xAF);
  writeCameraRegister(0x87, 0xC4);
  writeCameraRegister(0x88, 0xD7);
  writeCameraRegister(0x89, 0xE8);
  writeCameraRegister(0x13, 0xC0);
  writeCameraRegister(0x00, 0x00);
  writeCameraRegister(0x10, 0x00);
  writeCameraRegister(0x0D, 0x40);
  writeCameraRegister(0x14, 0x18);
  writeCameraRegister(0xA5, 0x05);
  writeCameraRegister(0xAB, 0x07);
  writeCameraRegister(0x24, 0x95);
  writeCameraRegister(0x25, 0x33);
  writeCameraRegister(0x26, 0xE3);
  writeCameraRegister(0x9F, 0x78);
  writeCameraRegister(0xA0, 0x68);
  writeCameraRegister(0xA1, 0x03);
  writeCameraRegister(0xA6, 0xD8);
  writeCameraRegister(0xA7, 0xD8);
  writeCameraRegister(0xA8, 0xF0);
  writeCameraRegister(0xA9, 0x90);
  writeCameraRegister(0xAA, 0x94);
  writeCameraRegister(0x13, 0xC5);
  writeCameraRegister(0x30, 0x00);
  writeCameraRegister(0x31, 0x00);
  writeCameraRegister(0x0E, 0x61);
  writeCameraRegister(0x0F, 0x4B);
  writeCameraRegister(0x16, 0x02);
  writeCameraRegister(0x1E, 0x07);
  writeCameraRegister(0x21, 0x02);
  writeCameraRegister(0x22, 0x91);
  writeCameraRegister(0x29, 0x07);
  writeCameraRegister(0x33, 0x0B);
  writeCameraRegister(0x35, 0x0B);
  writeCameraRegister(0x37, 0x1D);
  writeCameraRegister(0x38, 0x71);
  writeCameraRegister(0x39, 0x2A);
  writeCameraRegister(0x3C, 0x78);
  writeCameraRegister(0x4D, 0x40);
  writeCameraRegister(0x4E, 0x20);
  writeCameraRegister(0x69, 0x00);
  writeCameraRegister(0x74, 0x10);
  writeCameraRegister(0x8D, 0x4F);
  writeCameraRegister(0x8E, 0x00);
  writeCameraRegister(0x8F, 0x00);
  writeCameraRegister(0x90, 0x00);
  writeCameraRegister(0x91, 0x00);
  writeCameraRegister(0x96, 0x00);
  writeCameraRegister(0x9A, 0x00);
  writeCameraRegister(0xB0, 0x84);
  writeCameraRegister(0xB1, 0x0C);
  writeCameraRegister(0xB2, 0x0E);
  writeCameraRegister(0xB3, 0x82);
  writeCameraRegister(0xB8, 0x0A);
  writeCameraRegister(0x43, 0x0A);
  writeCameraRegister(0x44, 0xF0);
  writeCameraRegister(0x45, 0x34);
  writeCameraRegister(0x46, 0x58);
  writeCameraRegister(0x47, 0x28);
  writeCameraRegister(0x48, 0x3A);
  writeCameraRegister(0x59, 0x88);
  writeCameraRegister(0x5A, 0x88);
  writeCameraRegister(0x5B, 0x44);
  writeCameraRegister(0x5C, 0x67);
  writeCameraRegister(0x5D, 0x49);
  writeCameraRegister(0x5E, 0x0E);
  writeCameraRegister(0x6C, 0x0A);
  writeCameraRegister(0x6D, 0x55);
  writeCameraRegister(0x6E, 0x11);
  writeCameraRegister(0x6F, 0x9E);
  writeCameraRegister(0x6A, 0x40);
  writeCameraRegister(0x01, 0x40);
  writeCameraRegister(0x02, 0x60);
  writeCameraRegister(0x13, 0xC7);
  writeCameraRegister(0x4F, 0x80);
  writeCameraRegister(0x50, 0x80);
  writeCameraRegister(0x51, 0x00);
  writeCameraRegister(0x52, 0x22);
  writeCameraRegister(0x53, 0x5E);
  writeCameraRegister(0x54, 0x80);
  writeCameraRegister(0x58, 0x9E);
  writeCameraRegister(0x41, 0x08);
  writeCameraRegister(0x3F, 0x00);
  writeCameraRegister(0x75, 0x05);
  writeCameraRegister(0x76, 0xE1);
  writeCameraRegister(0x4C, 0x00);
  writeCameraRegister(0x77, 0x01);
  writeCameraRegister(0x3D, 0x48);
  writeCameraRegister(0x4B, 0x09);
  writeCameraRegister(0xC9, 0x60);
  writeCameraRegister(0x56, 0x40);
  writeCameraRegister(0x34, 0x11);
  writeCameraRegister(0x3B, 0x12);
  writeCameraRegister(0xA4, 0x82);
  writeCameraRegister(0x96, 0x00);
  writeCameraRegister(0x97, 0x30);
  writeCameraRegister(0x98, 0x20);
  writeCameraRegister(0x99, 0x30);
  writeCameraRegister(0x9A, 0x84);
  writeCameraRegister(0x9B, 0x29);
  writeCameraRegister(0x9C, 0x03);
  writeCameraRegister(0x9D, 0x4C);
  writeCameraRegister(0x9E, 0x3F);
  writeCameraRegister(0x78, 0x04);
  writeCameraRegister(0x79, 0x01);
  writeCameraRegister(0xC8, 0xF0);
  writeCameraRegister(0x79, 0x0F);
  writeCameraRegister(0xC8, 0x00);
  writeCameraRegister(0x79, 0x10);
  writeCameraRegister(0xC8, 0x7E);
  writeCameraRegister(0x79, 0x0A);
  writeCameraRegister(0xC8, 0x80);
  writeCameraRegister(0x79, 0x0B);
  writeCameraRegister(0xC8, 0x01);
  writeCameraRegister(0x79, 0x0C);
  writeCameraRegister(0xC8, 0x0F);
  writeCameraRegister(0x79, 0x0D);
  writeCameraRegister(0xC8, 0x20);
  writeCameraRegister(0x79, 0x09);
  writeCameraRegister(0xC8, 0x80);
  writeCameraRegister(0x79, 0x02);
  writeCameraRegister(0xC8, 0xC0);
  writeCameraRegister(0x79, 0x03);
  writeCameraRegister(0xC8, 0x40);
  writeCameraRegister(0x79, 0x05);
  writeCameraRegister(0xC8, 0x30);
  writeCameraRegister(0x79, 0x26);
  writeCameraRegister(0xFF, 0xFF);
  writeCameraRegister(0x15, 0x20);
  writeCameraRegister(0x0C, 0x08); // was 04
  writeCameraRegister(0x3E, 0x19);
  writeCameraRegister(0x72, 0x11);
  writeCameraRegister(0x73, 0xF1);
  writeCameraRegister(0x17, 0x16);
  writeCameraRegister(0x18, 0x04);
  writeCameraRegister(0x32, 0xA4);
  writeCameraRegister(0x19, 0x02);
  writeCameraRegister(0x1A, 0x7A);
  writeCameraRegister(0x03, 0x0A);
  writeCameraRegister(0xFF, 0xFF);
  writeCameraRegister(0x12, 0x08); // was 00
  writeCameraRegister(0x8C, 0x00);
  writeCameraRegister(0x04, 0x00);
  writeCameraRegister(0x40, 0xC0);
  writeCameraRegister(0x14, 0x6A);
  writeCameraRegister(0x4F, 0x80);
  writeCameraRegister(0x50, 0x80);
  writeCameraRegister(0x51, 0x00);
  writeCameraRegister(0x52, 0x22);
  writeCameraRegister(0x53, 0x5E);
  writeCameraRegister(0x54, 0x80);
  writeCameraRegister(0x3D, 0x40);
  writeCameraRegister(0xFF, 0xFF);
  writeCameraRegister(0x11, 0b0010); // clock was 1F
  
  writeCameraRegister(0x3E, 0x19);
  writeCameraRegister(0x0C, 0x08);  // was one higher
  writeCameraRegister(0x73, 0xF1);
  writeCameraRegister(0x12, 0x08); // was 0x10

  _CP0_SET_COUNT(0);
 while(_CP0_GET_COUNT()<48000000/2){}

}

void writeCameraRegister(unsigned char reg, unsigned char val){
    i2c_master_start();
    i2c_master_send(OV7670_SCCB_ADDRESS<<1);
    i2c_master_send(reg);
    i2c_master_send(val);
    i2c_master_stop();
}

int ov7670_count_horz(unsigned char * d){  
    int pclk = 0;
    int old_pclk = 0;
    int new_pclk = 0;
    //A8 as INT3/PCLK, B13 as INT2/HREF, C9 as INT1/VSYNC
    
    while(PORTCbits.RC9 == 0){}
    while(PORTCbits.RC9 == 1){} // got  new image
        
    while(PORTBbits.RB13 == 0){}
    while(PORTBbits.RB13 == 1){}//1
    while(PORTBbits.RB13 == 0){}
    while(PORTBbits.RB13 == 1){}//2
    while(PORTBbits.RB13 == 0){}
    while(PORTBbits.RB13 == 1){}//3
        
    pclk = 0;
    while(PORTBbits.RB13 == 0){}
    while(PORTBbits.RB13 == 1){ // the 4th row
        new_pclk = PORTAbits.RA8;
        if(old_pclk == 1 & new_pclk == 0){
            d[pclk] = PORTC;
            pclk++;
        }
        old_pclk = new_pclk;
    }
        
    return pclk;
}

int ov7670_count_vert(unsigned char * d){
    int rowclk = 0;
    int pclk = 0;
    int rowcount = 0;
    int old_rowclk = 0;
    int new_rowclk = 0;
    //A8 as INT3/PCLK, B13 as INT2/HREF, C9 as INT1/VSYNC
    
    while(PORTCbits.RC9 == 0){}
    while(PORTCbits.RC9 == 1){} // got a new image
    
    while(rowcount < 200){
        while(PORTBbits.RB13 == 0){}
        rowclk = 0;
        while(PORTBbits.RB13 == 1){ // wait for row to start
            new_rowclk = PORTAbits.RA8;
            if(old_rowclk == 1 & new_rowclk == 0){ // pixel clock changes
                rowclk++; // how many pixels you've seen
                if (rowclk == 100 | rowclk == 101) { // remember this pixel, brightness or color
                    pclk++;
                    d[pclk] = PORTC;
                }
            }
            old_rowclk = new_rowclk;
        }
        rowcount++;
    }
        
    return pclk;
}