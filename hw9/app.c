#include "app.h"
#include "ili9341_PIC32.h"
APP_DATA appData;
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <stdio.h>
//#include <ili9341.c>
#define  configRead 0b11010111 
#define  configWrite 0b11010110
#define  WHO_AM_I 0b00001111
#define configAcc 0b10000010
#define  CTRL1_XL 0b00010000
#define CTRL2_G 0b00010001
#define configG 0b10001000
#define OUTX_L_XL  0b00101000
#define OUTX_H_XL 0b00101001

#define centerx 120
#define centery 160
#define framelength 200
#define framewidth 2

void LCD_bar(unsigned short x, unsigned short y, unsigned short L, unsigned short H, unsigned short barcolor, unsigned short framecolor);
void LCD_get(unsigned short x, unsigned short y, char *letter, unsigned lettercolor, unsigned bgcolor);
void i2c_master_setup(void) ;
void i2c_master_start(void) ;
void i2c_master_restart(void) ;
void i2c_master_send(unsigned char byte) ;
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);
void I2C_read_multiple(unsigned char init_add, unsigned char init_reg,unsigned char *data,int length);
void LCD_bar_right(unsigned short x, unsigned short y, unsigned short L, unsigned short H,unsigned short V, unsigned short barcolor, unsigned short framecolor);
void LCD_bar_left(unsigned short x, unsigned short y, unsigned short L, unsigned short H,unsigned short V, unsigned short barcolor, unsigned short framecolor);
void LCD_bar_up(unsigned short x, unsigned short y, unsigned short L, unsigned short H,unsigned short V, unsigned short barcolor, unsigned short framecolor);
void LCD_bar_down(unsigned short x, unsigned short y, unsigned short L, unsigned short H,unsigned short V, unsigned short barcolor, unsigned short framecolor);

void APP_Initialize ( void )
{
    
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
    TRISAbits.TRISA4 = 0; //A4 Output
    TRISBbits.TRISB4 = 1; //B4 Input 
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
   // TRISBbits.TRISB2 = 0;
    //TRISBbits.TRISB3 = 0;
    __builtin_enable_interrupts();
    i2c_master_setup();
    SPI1_init(); // initial SPI
    LCD_init(); //Initial LCD
    LCD_clearScreen(ILI9341_DARKCYAN); //LCD background to black

 
    
    _CP0_SET_COUNT(0);
   
    
}



void APP_Tasks ( void )
{
     
  /////// k=0;
    short at_x;
    float ac_x;
    short at_y;
    float ac_y;
    short at_z;
    float ac_z;
    short temp_temp;
    float temp;
       unsigned char who;
    unsigned char a;
    unsigned char b=0;
    unsigned char data[13];
    unsigned char *index=data;
    int Lx;
    int Ly;
    
    char letter[100];
    double f;
    int k=0;
    int check_1s;///////////////////////////////////////////////////////////////////
   
       i2c_master_start();
       i2c_master_send(configWrite);
       i2c_master_send(CTRL1_XL);
       i2c_master_send(configAcc);
       i2c_master_stop;
       
       i2c_master_start();
       i2c_master_send(configWrite);
       i2c_master_send(CTRL2_G);
       i2c_master_send(configG);
       i2c_master_stop;
       
   
       i2c_master_start();
       i2c_master_send(configWrite);
       i2c_master_send(WHO_AM_I);
       i2c_master_stop();
       i2c_master_restart();
       i2c_master_send(configRead);
       who=i2c_master_recv();
       i2c_master_ack(1);
       i2c_master_stop();
     
  

        
       if  (_CP0_GET_COUNT()>12000)
        {    
        f= 5*24000000.0/_CP0_GET_COUNT();
        sprintf(letter,"who %d",who);   
        LCD_get(10, 10,letter,ILI9341_WHITE,ILI9341_DARKCYAN);      
        sprintf(letter,"ax %.2f",ac_x);   
        LCD_get(10, 18,letter,ILI9341_WHITE,ILI9341_DARKCYAN); 
        sprintf(letter,"ay %.2f",ac_y);   
        LCD_get(10, 26,letter,ILI9341_WHITE,ILI9341_DARKCYAN);
        sprintf(letter,"az %.2f",ac_z);   
        LCD_get(10, 34,letter,ILI9341_WHITE,ILI9341_DARKCYAN); 
        /*sprintf(letter,"Lx %d",Lx);   
        LCD_get(10, 42,letter,ILI9341_WHITE,ILI9341_DARKCYAN); 
        sprintf(letter,"Ly %d",Ly);   
        LCD_get(10, 50,letter,ILI9341_WHITE,ILI9341_DARKCYAN); 
        //LCD_get(10, 32,letter,ILI9341_WHITE,ILI9341_BLACK);
        k++;*/
        _CP0_SET_COUNT(0);
      LATAbits.LATA4 =!LATAbits.LATA4 ;
     I2C_read_multiple(0,0x20,index,14);
    // LCD_bar_right(centerx, centery, framewidth,framewidth,framewidth, ILI9341_BLACK, ILI9341_WHITE); //center    
    // LCD_bar_right(centerx+framewidth, centery, framelength,framewidth, 10,ILI9341_BLACK, ILI9341_WHITE); //right 
    // LCD_bar_left(centerx, centery,framelength,framewidth,10,ILI9341_BLACK, ILI9341_WHITE); //left
     //LCD_bar_up(centerx, centery, framelength,framewidth,10, ILI9341_BLACK, ILI9341_WHITE); //up
    // LCD_bar_down(centerx, centery+framewidth, framelength,framewidth,10, ILI9341_BLACK, ILI9341_WHITE); //down
        }
        
        at_x=data[9]<<8|data[8];
        ac_x=4*9.8/(65536)*at_x;
        
        at_y=data[11]<<8|data[10];
        ac_y=4*9.8/(65536)*at_y;
        
        at_z=data[13]<<8|data[12];
        ac_z=4*9.8/(65536)*at_z;
        
       temp_temp=data[1]<<8||data[0];
       temp=temp_temp/65535*135.0-40;
               
       if (ac_x>0)
    {
        if(ac_y>0)
        {
            Lx=20*(ac_x);
            Ly=20*(ac_y);
             LCD_bar_right(centerx, centery, Lx,framewidth, 10,ILI9341_BLACK, ILI9341_BLACK);
             LCD_bar_down(centerx, centery+framewidth, Ly,framewidth,10, ILI9341_BLACK, ILI9341_BLACK); //
             LCD_bar_right(centerx+framewidth+Lx, centery,  framelength,framewidth, 10,ILI9341_BLACK, ILI9341_WHITE);
             LCD_bar_down(centerx, centery+Ly+framewidth,  framelength,framewidth,10, ILI9341_BLACK, ILI9341_WHITE); //upup
             LCD_bar_left(centerx, centery,framelength,framewidth,10,ILI9341_BLACK, ILI9341_WHITE); 
             LCD_bar_up(centerx, centery, framelength,framewidth,10, ILI9341_BLACK, ILI9341_WHITE); 
             
        }
    }
     
       if (ac_x<0)
    {
        if(ac_y>0)
        {
            Lx=-20*(ac_x);
            Ly=20*(ac_y);
             LCD_bar_left(centerx+framewidth, centery, Lx,framewidth, 10,ILI9341_BLACK, ILI9341_BLACK);
             LCD_bar_down(centerx, centery+framewidth, Ly,framewidth,10, ILI9341_BLACK, ILI9341_BLACK); //
             LCD_bar_left(centerx-framewidth-Lx, centery,  framelength,framewidth, 10,ILI9341_BLACK, ILI9341_WHITE);
             LCD_bar_down(centerx, centery+Ly+framewidth,  framelength,framewidth,10, ILI9341_BLACK, ILI9341_WHITE); //upup
             LCD_bar_right(centerx+framewidth, centery,framelength,framewidth,10,ILI9341_BLACK, ILI9341_WHITE); 
             LCD_bar_up(centerx, centery, framelength,framewidth,10, ILI9341_BLACK, ILI9341_WHITE); 
        }
    }
     
       if (ac_x<0)
    {
        if(ac_y<0)
        {
            Lx=-20*(ac_x);
            Ly=-20*(ac_y);
             LCD_bar_left(centerx+framewidth, centery, Lx,framewidth, 10,ILI9341_BLACK, ILI9341_BLACK);
             LCD_bar_up(centerx, centery, Ly,framewidth,10, ILI9341_BLACK, ILI9341_BLACK); //
             LCD_bar_left(centerx-framewidth-Lx, centery,  framelength,framewidth, 10,ILI9341_BLACK, ILI9341_WHITE);
             LCD_bar_up(centerx, centery-Ly,  framelength,framewidth,10, ILI9341_BLACK, ILI9341_WHITE); //upup
             LCD_bar_right(centerx+framewidth, centery,framelength,framewidth,10,ILI9341_BLACK, ILI9341_WHITE); 
             LCD_bar_down(centerx, centery+framewidth, framelength,framewidth,10, ILI9341_BLACK, ILI9341_WHITE); 
        }
    }
       
       
    if (ac_x>0)
    {
        if(ac_y<0)
        {
            Lx=20*(ac_x);
            Ly=-20*(ac_y);
             LCD_bar_right(centerx, centery, Lx,framewidth, 10,ILI9341_BLACK, ILI9341_BLACK);
             LCD_bar_up(centerx, centery, Ly,framewidth,10, ILI9341_BLACK, ILI9341_BLACK); //
             LCD_bar_right(centerx+framewidth+Lx, centery,  framelength,framewidth, 10,ILI9341_BLACK, ILI9341_WHITE);
             LCD_bar_up(centerx, centery-Ly,  framelength,framewidth,10, ILI9341_BLACK, ILI9341_WHITE); //upup
             LCD_bar_left(centerx, centery,framelength,framewidth,10,ILI9341_BLACK, ILI9341_WHITE); 
             LCD_bar_down(centerx, centery+framewidth, framelength,framewidth,10, ILI9341_BLACK, ILI9341_WHITE); 
        }
    }
     
     
    
        /*            LATAbits.LATA4==0; // initial value assigned zero
               if (PORTBbits.RB4==0)
               {
                   LATAbits.LATA4=0;   // While button pressed, led disabled 
               }
               if (PORTBbits.RB4==1)
               if(_CP0_GET_COUNT()>=1200000)
               {
                    LATAbits.LATA4 = !LATAbits.LATA4; // Every 0.5ms Filp LED voltage 
                    _CP0_SET_COUNT(0);   // Clear Clk Timer
               }
        */             

    

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 
void I2C_read_multiple(unsigned char init_add, unsigned char init_reg,unsigned char *index,int length)
{
    int i;
for (i=init_add;i<length;i++)
{
       i2c_master_start();
       i2c_master_send(configWrite);
       i2c_master_send(init_reg+i);
       i2c_master_stop();
       i2c_master_restart();
       i2c_master_send(configRead);
       *index=i2c_master_recv();
       i2c_master_ack(1);
       i2c_master_stop();
       index++;
   
}
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
  SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
  RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
  TRISBbits.TRISB7 = 0; // CS is B7
  CS = 1; // CS starts high

  // DC pin
  TRISBbits.TRISB15 = 0;
  DC = 1;
  
  SPI1CON = 0; // turn off the spi module and reset it
  SPI1BUF; // clear the rx buffer by reading from it
  SPI1BRG = 0; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
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

void LCD_letter(unsigned short x, unsigned short y, char letter, unsigned lettercolor, unsigned bgcolor){
    char rowloc = letter - 0x20;
    int row;
    int column;
    for (column=0; column <= 4;)
    {
        char pix = ASCII [rowloc][column];
        for (row= 0;row <= 7;){
            
            if (x+column <= ILI9341_TFTWIDTH && y+row<= ILI9341_TFTHEIGHT){
                if (pix >> row & 1 == 1){
                    LCD_drawPixel (x+column,y+row,lettercolor);
                }
                    else{
                    LCD_drawPixel (x+column,y+row,bgcolor);
                }
            }
           row++;
        }
           column++;
    }
}

void LCD_get(unsigned short x, unsigned short y, char *letter, unsigned lettercolor, unsigned bgcolor){ 
    int i = 0;
    while(letter[i] != '\0'){
        LCD_letter(x + i * 5,y,letter[i],lettercolor,bgcolor);
        i++;
    }
}

void LCD_bar(unsigned short x, unsigned short y, unsigned short L, unsigned short H, unsigned short barcolor, unsigned short framecolor){
    int i,j;
    for(i = 0; i < L; ){
        i++;
        for(j = 0; j < H;){
            j++;
            LCD_drawPixel(x + i, y + j, barcolor);
        }
    }
    if (L < 100){
        for(i = L; i < 100;){
            i++;
            for(j = L; j< H;){
                j++;
                LCD_drawPixel(x + i,y + j,framecolor);
            }
        }
    }
}


void i2c_master_setup(void) {
  I2C2BRG = 37;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2  50Mhz 
                                    // look up PGD for your PIC32 PGD=104ns
  I2C2CONbits.ON = 1;               // turn on the I2C1 module
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C2CONbits.RSEN = 1;           // send a restart 
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C2TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
    // ("I2C2 Master: failed to receive ACK\r\n");
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }   
  // wait for STOP to complete
}

void LCD_bar_right(unsigned short x, unsigned short y, unsigned short L, unsigned short H,unsigned short V, unsigned short barcolor, unsigned short framecolor){
    int i,j,l,m;
        for(i = 0; i < L;i++){
             for(j = 0; j< H;j++){
                LCD_drawPixel(x + i,y + j,framecolor);
           }
        }

   
}
void LCD_bar_left(unsigned short x, unsigned short y, unsigned short L, unsigned short H,unsigned short V, unsigned short barcolor, unsigned short framecolor){
    int i,j,l,m;
        for(i = 0; i < L;i++){
             for(j = 0; j< H;j++){
                LCD_drawPixel(x - i,y + j,framecolor);
           }
        }

   
}
void LCD_bar_up(unsigned short x, unsigned short y, unsigned short L, unsigned short H,unsigned short V, unsigned short barcolor, unsigned short framecolor){
    int i,j,l,m;
        for(j = 0; j < L;j++){
             for(i = 0; i< H;i++){
                LCD_drawPixel(x + i,y - j,framecolor);
           }
        }

}
void LCD_bar_down(unsigned short x, unsigned short y, unsigned short L, unsigned short H,unsigned short V, unsigned short barcolor, unsigned short framecolor){
    int i,j,l,m;
        for(j = 0; j < L;j++){
             for(i = 0; i< H;i++){
               LCD_drawPixel(x + i,y + j,framecolor);
           }
        }
}

/*******************************************************************************
 End of File
 */
