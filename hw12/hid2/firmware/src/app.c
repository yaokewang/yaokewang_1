
#include "app.h"
#include "ili9341_PIC32.h"


APP_DATA appData;

/* Mouse Report */
MOUSE_REPORT mouseReport APP_MAKE_BUFFER_DMA_READY;
MOUSE_REPORT mouseReportPrevious APP_MAKE_BUFFER_DMA_READY;

#define  configRead 0b11010111 
#define  configWrite 0b11010110
#define  WHO_AM_I 0b00001111
#define configAcc 0b10000010
#define  CTRL1_XL 0b00010000
#define CTRL2_G 0b00010001
#define configG 0b10001000
#define OUTX_L_XL  0b00101000
#define OUTX_H_XL 0b00101001

short ax_raw;
float ax_real;
short gx_raw;
float gx_real;
short ay_raw;
float ay_real;
short gy_raw;
float gy_real;
short az_raw;
float az_real;
short gz_raw;
float gz_real;
     int  movement_length;
unsigned char data[13];
unsigned char *index=data;

void i2c_master_setup(void) ;
void i2c_master_start(void) ;
void i2c_master_restart(void) ;
void i2c_master_send(unsigned char byte) ;
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);
void I2C_read_multiple(unsigned char init_add, unsigned char init_reg,unsigned char *data,int length);

 int m_x;
 int m_y;
 int treash = 1;
 
void APP_USBDeviceHIDEventHandler(USB_DEVICE_HID_INDEX hidInstance,
        USB_DEVICE_HID_EVENT event, void * eventData, uintptr_t userData) {
    APP_DATA * appData = (APP_DATA *) userData;

    switch (event) {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* This means the mouse report was sent.
             We are free to send another report */

            appData->isMouseReportSendBusy = false;
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* Dont care for other event in this demo */
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* save Idle rate received from Host */
            appData->idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*) eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->idleRate), 1);

            /* On successfully receiving Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function driver returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;

        case USB_DEVICE_HID_EVENT_SET_PROTOCOL:
            /* Host is trying set protocol. Now receive the protocol and save */
            appData->activeProtocol = *(USB_HID_PROTOCOL_CODE *) eventData;

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_HID_EVENT_GET_PROTOCOL:

            /* Host is requesting for Current Protocol. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->activeProtocol), 1);

            /* On successfully receiving Idle rate, the Host would acknowledge
              back with a Zero Length packet. The HID function driver returns
              an event USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the
              application upon receiving this Zero Length packet from Host.
              USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates
              this control transfer event is complete */
            break;

        case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;

        default:
            break;
    }
}

/*******************************************************************************
  Function:
    void APP_USBDeviceEventHandler (USB_DEVICE_EVENT event,
        USB_DEVICE_EVENT_DATA * eventData)
  Summary:
    Event callback generated by USB device layer.
  Description:
    This event handler will handle all device layer events.
  Parameters:
    None.
  Returns:
    None.
 */

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED * configurationValue;
    switch (event) {
        case USB_DEVICE_EVENT_SOF:
            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            appData.setIdleTimer++;
            break;
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Device got deconfigured */

            appData.isConfigured = false;
            appData.isMouseReportSendBusy = false;
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            //appData.emulateMouse = true;
            //BSP_LEDOn ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2 );
            //BSP_LEDOff ( APP_USB_LED_3 );

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Device is configured */
            configurationValue = (USB_DEVICE_EVENT_DATA_CONFIGURED *) eventData;
            if (configurationValue->configurationValue == 1) {
                appData.isConfigured = true;

                //BSP_LEDOff ( APP_USB_LED_1 );
                //BSP_LEDOff ( APP_USB_LED_2 );
                //BSP_LEDOn ( APP_USB_LED_3 );

                /* Register the Application HID Event Handler. */

                USB_DEVICE_HID_EventHandlerSet(appData.hidInstance,
                        APP_USBDeviceHIDEventHandler, (uintptr_t) & appData);
            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            //BSP_LEDOff ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2 );
            //BSP_LEDOn ( APP_USB_LED_3 );
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;

    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;
    appData.isConfigured = false;
    //appData.emulateMouse = true;
    appData.hidInstance = 0;
    appData.isMouseReportSendBusy = false;
      ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    TRISAbits.TRISA4 = 0; //   Set A4 as a output port
    TRISBbits.TRISB4 = 1; //   Set B4 as a input port
    i2c_master_setup(); //initial i2c


    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
     

}



/******************************************************************************
  Function:
    void APP_Tasks ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle,
                        APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }
            break;
        }

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured. The 
             * isConfigured flag is updated in the
             * Device Event Handler */

            if (appData.isConfigured) {
                appData.state = APP_STATE_MOUSE_EMULATE;
            }
            break;

        case APP_STATE_MOUSE_EMULATE:
            
            // every 50th loop, or 20 times per second
            
                 
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
       

       
       I2C_read_multiple(0,0x20,index,14);      
       ax_raw=data[9]<<8|data[8];
       ax_real=4*9.81/65536*ax_raw;
       ay_raw=data[11]<<8|data[10];
       ay_real=-4*9.81/65536*ay_raw;
       az_raw=data[13]<<8|data[12];
       az_real=4*9.81/65536*az_raw;
       //Gyro Read
       gx_raw=data[3]<<8|data[2];
       gx_real=gx_raw*1000*2/65536;
       gy_raw=data[5]<<8|data[4];
       gy_real=gy_raw*1000*2/65536;
       gz_raw=data[7]<<8|data[6];
       gz_real=gz_raw*1000*2/65536;     
   

      
        
       m_x=0.5*ax_real*treash;
       m_y=0.5*ay_real*treash;
            
                if (movement_length > 9) {
            
                appData.mouseButton[0] = MOUSE_BUTTON_STATE_RELEASED;
                appData.mouseButton[1] = MOUSE_BUTTON_STATE_RELEASED;
                if (m_x>treash||m_x<-treash){
                appData.xCoordinate = (int8_t) m_x;
                }
                if (m_y>treash ||m_y<-treash){
                appData.yCoordinate = (int8_t) -m_y;
                }
                if (m_x<=treash && m_x>=-treash){
                appData.xCoordinate = (int8_t) 0;
                }
                if (m_y<=treash && m_y>=-treash){
                appData.yCoordinate = (int8_t) 0;   
               }
               // vector++;
                movement_length = 0;
            }
         
      

            if (!appData.isMouseReportSendBusy) {
                /* This means we can send the mouse report. The
                   isMouseReportBusy flag is updated in the HID Event Handler. */

                appData.isMouseReportSendBusy = true;

                /* Create the mouse report */

                MOUSE_ReportCreate(appData.xCoordinate, appData.yCoordinate,
                        appData.mouseButton, &mouseReport);

                if (memcmp((const void *) &mouseReportPrevious, (const void *) &mouseReport,
                        (size_t)sizeof (mouseReport)) == 0) {
                    /* Reports are same as previous report. However mouse reports
                     * can be same as previous report as the coordinate positions are relative.
                     * In that case it needs to be sent */
                    if ((appData.xCoordinate == 0) && (appData.yCoordinate == 0)) {
                        /* If the coordinate positions are 0, that means there
                         * is no relative change */
                        if (appData.idleRate == 0) {
                            appData.isMouseReportSendBusy = false;
                        } else {
                            /* Check the idle rate here. If idle rate time elapsed
                             * then the data will be sent. Idle rate resolution is
                             * 4 msec as per HID specification; possible range is
                             * between 4msec >= idlerate <= 1020 msec.
                             */
                            if (appData.setIdleTimer
                                    >= appData.idleRate * 4) {
                                /* Send REPORT as idle time has elapsed */
                                appData.isMouseReportSendBusy = true;
                            } else {
                                /* Do not send REPORT as idle time has not elapsed */
                                appData.isMouseReportSendBusy = false;
                            }
                        }
                    }

                }
                if (appData.isMouseReportSendBusy == true) {
                    /* Copy the report sent to previous */
                    memcpy((void *) &mouseReportPrevious, (const void *) &mouseReport,
                            (size_t)sizeof (mouseReport));

                    /* Send the mouse report. */
                    USB_DEVICE_HID_ReportSend(appData.hidInstance,
                            &appData.reportTransferHandle, (uint8_t*) & mouseReport,
                            sizeof (MOUSE_REPORT));
                    appData.setIdleTimer = 0;
                }
        movement_length++;
            }

            break;

        case APP_STATE_ERROR:

            break;

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
