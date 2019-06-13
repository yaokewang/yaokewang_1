#include <xc.h>
#include "ili9341.h"
#include <sys/attribs.h>

void LCD_init() {
    SPI1_init();
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

void LCD_takechar(unsigned short x, unsigned short y, char msg, unsigned short color1, unsigned short color2){
    char row = msg - 0x20;
    int bit;
    int col = 0;
    for (col = 0; col < 5; col++){
        char pixel = ASCII[row][col];
        for(bit = 0;bit < 8;bit++){
            if(x+col < ILI9341_TFTWIDTH && y+bit < ILI9341_TFTHEIGHT){
                if(pixel >> bit & 1 == 1){
                    LCD_drawPixel(x + col, y + bit, color1);
                }
                else{
                    LCD_drawPixel(x + col,y + bit, color2);
                }
            }
        }
    }
}

void LCD_getstring(unsigned short x, unsigned short y, char *msg, unsigned short color1, unsigned short color2){
    int i = 0;
    while(msg[i] != '\0'){
        LCD_takechar(x + i * 5,y,msg[i],color1,color2);
        i++;
    }
}

void LCD_drawbar(unsigned short x, unsigned short y, unsigned short height, unsigned short length, unsigned short color1, unsigned short color2){
    int i,j;
    for(i = 0; i < length; i++){
        for(j = 0; j < height; j++){
            LCD_drawPixel(x + i, y + j, color1);
        }
    }
    if (length < 100){
        for(i = length; i < 100; i ++){
            for(j = length; j< 100;j++){
                LCD_drawPixel(x + i,y + j,color2);
            }
        }
    }
}

void LCD_staticbar(unsigned short x, unsigned short y, unsigned short height, unsigned short length, unsigned color1){
    int i, j;
    for (i = 0; i < length; i++){
        for (j = 0; j < height;j++){
            LCD_drawPixel(x + i, y + j, color1);
        }
    }
}

void LCD_xbar(unsigned short x, unsigned short y, unsigned short height, float xacc, unsigned color1, unsigned color2){
    int i, j;
    int length = (int) (abs(xacc * 45));
    if (xacc > 0){
        for (i = 0; i < length; i++){
            for (j = 0; j < height; j++){
                LCD_drawPixel(x + i,y + j, color1);
            }
        }
        if (length < 45){
            for (i = length; i < 45; i++){
                for (j = length; j < height; j++){
                    LCD_drawPixel(x + i, y + j,color2);
            }
           }
        }
    }else{
        for (i = 0; i < length; i++){
            for (j = 0; j < height; j++){
                LCD_drawPixel(x - i, y + j,color1);
            }
        }
        if (length < 45){
            for ( i = length; i < 45; i++){
                for(j = length; j < height; j++){
                    LCD_drawPixel(x - i, y + j, color2);
                }
            }
        }
    }              
    }

void LCD_ybar(unsigned short x, unsigned short y, float yacc, unsigned short length, unsigned color1, unsigned color2){
    int i, j;
    int height = (int) (abs(yacc * 45));
    if (yacc > 0){
        for (i = 0; i < length; i++){
            for (j = 0; j < height; j++){
                LCD_drawPixel(x + i,y + j, color1);
            }
        }
        if (height < 45){
            for (i = height; i < length; i++){
                for (j = height; j < 45; j++){
                    LCD_drawPixel(x + i, y + j,color2);
            }
           }
        }
    }else{
        for (i = 0; i < length; i++){
            for (j = 0; j < height; j++){
                LCD_drawPixel(x + i, y - j,color1);
            }
        }
        if (length < 45){
            for ( i = height; i < length; i++){
                for(j = height; j < 45; j++){
                    LCD_drawPixel(x + i, y - j, color2);
                }
            }
        }
    }              
    }

void LCD_plot(unsigned short x, unsigned short y, unsigned short c, unsigned char *msg, int length){
    CS = 0;
    int i;
    for(i = 0;i <length; i++){
        LCD_setAddr(x-(msg[i]>>3),y-i,1,1);
        LCD_data16(c);
   
    }
    CS = 1;
    
}

void LCD_axis(unsigned short x, unsigned short y, unsigned short height, unsigned length, unsigned short c){
    CS = 0;
    int i, j;
    for(i = 0; i<height; i++) //start at x and go until x+w
    {
       
        LCD_setAddr(x,y-i,1,1); //find pixel location
        LCD_data16(c); //sets a color
    }
    for (j = 0; j<length; j++)
    {
        LCD_setAddr(x-j, y, 1, 1);
        LCD_data16(c);
    }
    CS = 1;
}

    
