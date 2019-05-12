#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <stdio.h>
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define  configRead 0b11010111 
#define  configWrite 0b11010110
#define  WHO_AM_I 0b00001111
#define configAcc 0b10000010
#define  CTRL1_XL 0b00010000
#define CTRL2_G 0b00010001
#define configG 0b10001000
#define OUTX_L_XL  0b00101000
#define OUTX_H_XL 0b00101001

int output_flag=0;


uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0;
int startTime = 0; // to remember the loop time

void i2c_master_setup(void) ;
void i2c_master_start(void) ;
void i2c_master_restart(void) ;
void i2c_master_send(unsigned char byte) ;
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);
void I2C_read_multiple(unsigned char init_add, unsigned char init_reg,unsigned char *data,int length);



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

 short  buffer[100];

unsigned char data[13];
unsigned char *index=data;

    unsigned char who;
    unsigned char a;
    unsigned char b=0;
  //  unsigned char data[13];
    //unsigned char *index=data;
 
    int k;
    
    
    int n_MAF=50;
    int n_FIR=25;
    
     float g_IIR_p=0;
     float g_IIR;

        float  coef_FIR[26] ={0.003074,0.004152,0.0067574,0.011166,
        0.017457,0.025479,0.034856,0.045009,
        0.055212,0.064667,0.07259,0.078297,
        0.081284,0.081284,0.078297,0.07259,
        0.064667,0.055212,0.045009,0.034856,
        0.025479,0.017457,0.011166,0.0067574,
        0.004152,0.003074,};
    
     float IIR_A=0.1;
     float IIR_B=0.9;



// *****************************************************************************
/* Application Data
  Summary:
    Holds application data
  Description:
    This structure holds the application's data.
  Remarks:
    This structure should be initialized by the APP_Initialize function.
    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

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

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
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

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

    /* PUT YOUR LCD, IMU, AND PIN INITIALIZATIONS HERE */

    startTime = _CP0_GET_COUNT();
    
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    TRISAbits.TRISA4 = 0; //   Set A4 as a output port
    TRISBbits.TRISB4 = 1; //   Set B4 as a input port
     i2c_master_setup();
     
 
    
   
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    
    
    

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
       
       
    
      
       for (k=99;k>0;k--)
       {
       buffer[k]=buffer[k-1];
       }
       buffer[0]=az_raw;
       
       float g_MAF=0;
       
       
       for (k=0;k<n_MAF;k++)
       {
          g_MAF=g_MAF+buffer[k];
       }
       g_MAF=1.0*g_MAF/n_MAF*9.81*4/65536; 
       
       
        float  g_FIR=0;
        for (k=0;k<n_FIR;k++)
       {
          g_FIR=1.0*g_FIR+coef_FIR[k]*buffer[k];
       }
        g_FIR=1.0* g_FIR*9.81*4/65536; 
      
        float w;
        
        w=9.81*4/65536*buffer[0]*IIR_A;
        g_IIR=(1.0*g_IIR_p*IIR_B+w);
       
       
          LATAbits.LATA4 = !LATAbits.LATA4;
       
     
      
       

    switch (appData.state) {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

      appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
		if (appData.isReadComplete == true) {
			appData.isReadComplete = false;
			appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

			USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
				&appData.readTransferHandle, appData.readBuffer,
				APP_READ_BUFFER_SIZE);
			if (strcmp(appData.readBuffer, "r")==0) {
				// We received an r, time to set sendDataFlag and set counter to zero
				output_flag = 1;
                i=0;

			}
			/* AT THIS POINT, appData.readBuffer[0] CONTAINS A LETTER
			THAT WAS SENT FROM THE COMPUTER */
			/* YOU COULD PUT AN IF STATEMENT HERE TO DETERMINE WHICH LETTER
			WAS RECEIVED (USUALLY IT IS THE NULL CHARACTER BECAUSE NOTHING WAS
		  TYPED) */

			if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
				appData.state = APP_STATE_ERROR;
				break;
			}
		}

		break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_TIMER:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

             /* WAIT FOR 5HZ TO PASS OR UNTIL A LETTER IS RECEIVED */
            if (appData.isReadComplete || _CP0_GET_COUNT() - startTime > (48000000 / 2 / 5)) {
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }


            break;


        case APP_STATE_SCHEDULE_WRITE:

            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

            
           
        if (appData.isReadComplete) { ; }
		if (output_flag == 1 && i<100 ) {
        	//len = sprintf(dataOut, "%d %d %.6f %.6f %.6f\r\n", i, gz_raw, g_MAF, g_FIR,g_IIR);
         //	len = sprintf(dataOut, "%d %.6f %.6f %.6f %.6f \r\n", i, az_real,g_MAF,g_FIR,g_IIR);
            len = sprintf(dataOut, "%d  %.6f %.6f %.6f %.6f %.6f %.6f \r\n", i, ax_real,ay_real,az_real,gx_real,gy_real,gz_real);
               g_IIR_p=g_IIR;
         //  len = sprintf(dataOut, "%d %d %d %d  \r\n", i, buffer[0],buffer[1],buffer[2]);
			//len = sprintf(dataOut, "%d" , output_flag);
            i++; // increment the index so we see a change in the text
			/* IF A LETTER WAS RECEIVED, ECHO IT BACK SO THE USER CAN SEE IT */

			/* ELSE SEND THE MESSAGE YOU WANTED TO SEND */

			USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
				&appData.writeTransferHandle,
				dataOut, len,
				USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
		}
		else {
			len = 1;
			dataOut[0] = 0;
			USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
				&appData.writeTransferHandle, dataOut, len,
				USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
			startTime = _CP0_GET_COUNT(); // reset the timer for acurate delays
		}
		break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
            
            
     
  
       
       
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


