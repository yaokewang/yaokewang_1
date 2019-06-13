#include <xc.h>
#include "imu.h"
#include <sys/attribs.h>

void imu_init(void){
    //turn off the analog input
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
   
    i2c_master_setup();
    
    //setup accelerometer
    i2c_master_start();
    i2c_master_send(IMU_ADDR << 1);
    i2c_master_send(0x10);
    i2c_master_send(0x82); // 1 0 0 0 0 0 1 0
    i2c_master_stop();
    
    //setup gyroscope
    i2c_master_start();
    i2c_master_send(IMU_ADDR << 1);
    i2c_master_send(0x11);
    i2c_master_send(0x88); //1 0 0 0 1 0 0 0
    i2c_master_stop();
    
    //setup IF_INC
    i2c_master_start();
    i2c_master_send(IMU_ADDR << 1);
    i2c_master_send(0x12);
    i2c_master_send(0x04); //0 0 0 0 0 1 0 0 
    i2c_master_stop();
}

void i2c_read_multiple(unsigned char address, unsigned char reg, unsigned char *data, int length){
    int i;
    i2c_master_start();
    i2c_master_send(IMU_ADDR << 1);
    i2c_master_send(0x20); //00100000
    i2c_master_restart();
    i2c_master_send(IMU_ADDR << 1 | 1);
    for (i = 0; i < length; i++){
        data[i] = i2c_master_recv();
        if(i != length - 1){
            i2c_master_ack(0);
        }
    }
    i2c_master_ack(1);
    i2c_master_stop();
    
}

unsigned char imu_test(void){
   unsigned char whoami;
  i2c_master_start();                   
  i2c_master_send(IMU_ADDR << 1);       
  i2c_master_send(0x0F);                

  i2c_master_restart();                 
  i2c_master_send(IMU_ADDR << 1 | 1);   
  whoami = i2c_master_recv();           
  i2c_master_ack(1);
  i2c_master_stop();
  return whoami;
}

float getX(unsigned char *data){
    signed short xacc = (data[9] << 8 | data[8]);
    return (xacc / 32767.0 * 2.0);
}

float getY(unsigned char *data){
    signed short yacc = (data[11] << 8 | data[10]);
    return (yacc / 32767.0 * 2.0);
}