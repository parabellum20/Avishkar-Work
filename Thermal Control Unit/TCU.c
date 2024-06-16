/* This Property belongs to Avishkar Hyperloop */

/* Author : Sai Suddhir  A B */


#include "ti/devices/msp432e4/driverlib/driverlib.h"

/* Standard Includes */
#include <ti/drivers/CAN.h>
#include "CAN_h.h"
#include <stdint.h>
#include <stdbool.h>
#include "uartstdio.h"
#include <string.h>
#include <stdbool.h>


/*define*/
#define Mdot 1.57
#define Cp 4181

/* System clock rate in Hz */
uint32_t sysClock;

/* CAN variables */
bool rxMsg = false;
bool errFlag = false;
uint32_t msgCount = 0;


uint8_t msgRxData[8] = {0x00};
//
///* Two CAN objects for Tx and One for Rx */
tCANMsgObject sCANTxMessage[2];
tCANMsgObject sCANRxMessage;

uint64_t msg2;

/* Scheduler variables */
uint8_t schedulerTimer;

typedef enum
{
    LIM_R,
    LIM_L,
    HVB_T,
    HVB_M,
    HVB_B
} PUMP;

bool Bit[20];

/* Function prototypes */
void configureUART(void);
void configureCAN(void);
void configureGPIOs(void);
void receiveCANMsgManager(void);
void configureSchedulerTimer(void);
void PUMP_ON(PUMP Pump_name);
void PUMP_OFF(PUMP Pump_name);
void MCU_transmit(uint8_t data[3]);
void Comparison(int arg1);
void flow_regulation(int arg1 , int arg2, float *LIM_ptr, float *BMS_ptr);
void Pump_control(float *LIM_ptr, float *BMS_ptr);

int i;


///*
//int main(void)
//{
//    /* Run from the PLL at 120 MHz */
//    sysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
//                SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
//                SYSCTL_CFG_VCO_480), 120000000);
//
//    /* Initialize the UART */
//    configureUART();
//
//    /* Initialize board external interrupts for SW1 and SW2 */
//    configureBUTTONS();
//
//    /* Initialize board LEDs D1, D2, D3 and D4 */
//    configureLEDS();
//
//    /* Initialize the CAN */
//    configureCAN();
//
//    /* CAN TX */
//    /* Initialize message objects to be able to send CAN message 1 */
//    sCANTxMessage[0].ui32MsgID = 0x200;
//    sCANTxMessage[0].ui32MsgIDMask = 0;                  /* No mask needed for TX  */
//    sCANTxMessage[0].ui32Flags = MSG_OBJ_TX_INT_ENABLE;  /* Enable interrupt on TX */
//    sCANTxMessage[0].ui32MsgLen = sizeof(msg1TxData);    /* Size of message        */
//    sCANTxMessage[0].pui8MsgData = msg1TxData;           /* Ptr to message content */
//
//    /* Initialize message objects to be able to send CAN message 2 */
//    sCANTxMessage[1].ui32MsgID = 0x201;
//    sCANTxMessage[1].ui32MsgIDMask = 0;                  /* No mask needed for TX  */
//   sCANTxMessage[1].ui32Flags = MSG_OBJ_TX_INT_ENABLE;  /* Enable interrupt on TX */
//    sCANTxMessage[1].ui32MsgLen = sizeof(msg2TxData);    /* Size of message        */
//    sCANTxMessage[1].pui8MsgData = msg2TxData;           /* Ptr to message content */
//
//    /* CAN RX */
//    /* Initialize message object 3 to be able to receive CAN message ID 0x300
//     * 0x301, 0x302 and 0x0303*/
//    sCANRxMessage.ui32MsgID = 0;
//    sCANRxMessage.ui32MsgIDMask = ~(0x303);              /* Look for specific ID's */
//    sCANRxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE      /* Enable interrupt on RX */
//            | MSG_OBJ_USE_ID_FILTER;                     /* and Filter ID          */
//    sCANRxMessage.ui32MsgLen = sizeof(msgRxData);        /* Size of message        */
//
//    /* Load the message object into the CAN peripheral. Once loaded an
//     * interrupt will occur only when a valid CAN ID is received. Use message
//     * object 3 for receiving messages */
//    MAP_CANMessageSet(CAN0_BASE, 3, &sCANRxMessage, MSG_OBJ_TYPE_RX);
//
//    /* Initialize the timer for the scheduler */
//    configureSchedulerTimer();
//
//
//    /* Loop forever */
//    while(1)
//    {
//        /*  10 ms */
//
//        if (schedulerTimer >= 1)
//        {
//            schedulerTimer = 0;
//            counter10ms++;
//
//            /* Update button status in the CAN message */
//            msg1TxData[1] = isSW1Pressed;
//            msg2TxData[1] = isSW2Pressed;
//
//            /* Send the CAN messages with the button status */
//            MAP_CANMessageSet(CAN0_BASE, 1, &sCANTxMessage[0], MSG_OBJ_TYPE_TX);
//            MAP_CANMessageSet(CAN0_BASE, 2, &sCANTxMessage[1], MSG_OBJ_TYPE_TX);
//        }
//
//        /* 20 ms */
//        if (counter10ms >= 2)
//        {
//            counter10ms = 0;
//            counter100ms++;
//
//            /* Process received messages */
//            receiveCANMsgManager();
//        }
//
//        /* 100 ms */
//        if (counter100ms >= 5)
//        {
//            counter100ms = 0;
//            counter1s++;
//        }
//
//        /* 1 sec */
//        if (counter1s >= 10)
//        {
//            counter1s = 0;
//
//            /* Toggle LED4  every 1 second */
//            toggleLED(LED4);
//        }
//
//        /* Check the error flag to see if errors occurred */
//        if(errFlag)
//        {
//            UARTprintf("error - cable connected?\n");
//            while(errFlag);
//        }
//    }z
//}

int main()
{
    sysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                    SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                    SYSCTL_CFG_VCO_480), 120000000);


    float LIM[3]={ecu.lim_thres.LIM_absMax, ecu.lim_thres.LIM_Max , ecu.lim_thres.LIM_Min};
    float BMS[3]={ecu.bms_thres.BMS_Max, ecu.bms_thres.BMS_Min , ecu.bms_thres.BMS_absMax};

    float *LIM_ptr = LIM;
    float *BMS_ptr = BMS;


    configureUART();
    configureGPIOs();
    configureCAN();

    configureSchedulerTimer();

    uint8_t prevschedulerTimer = 0;

    while(1)
    {

        if(schedulerTimer != prevschedulerTimer)
        {
            prevschedulerTimer=schedulerTimer;

            for(i=0;i<20;i++)
                {
                    Bit[i] = false;
                }

            receiveCANMsgManager();

            Pump_control(LIM_ptr,BMS_ptr);

            for(i=0;i<5;i++)
            {
                Comparison(i);
            }

            uint8_t msg1_TxData[3] = {0x00};
            for (i = 0; i < 8; i++) {
                if (Bit[i]) {
                    msg1_TxData[1] |= (1 << i);
                }
            }

            for (i = 0; i < 8; i++) {
                if (Bit[i + 8]) {
                    msg1_TxData[0] |= (1 << i);
                }
            }


            for (i = 0; i < 4; i++) {
                if (Bit[i + 16]) {
                    msg1_TxData[2] |= (1 << i);
                }
            }

            MCU_transmit(msg1_TxData);
     return 0;
          }
     }
}

void Pump_control(float *LIM_ptr, float *BMS_ptr)
{
    flow_regulation(0,0,LIM_ptr,BMS_ptr);
    flow_regulation(0,1,LIM_ptr,BMS_ptr);
//    flow_regulation(BMS_ptr,0);
//    flow_regulation(BMS_ptr,1);
//    flow_regulation(BMS_ptr,2);

}

void flow_regulation(int arg1 , int arg2, float *LIM_ptr, float *BMS_ptr)
{

    PUMP LIM_P;
//    PUMP BMS_P;
    uint32_t Temp;
    uint32_t GPIO;

    float Reservoir_temp = (ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_1a
                            +  ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_1b
                            + ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_2a
                            + ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_2b)/4 ;
    float Thresh[3];

    if (arg1 == 0)
    {
        if (arg2 == 0)
        {
            LIM_P = LIM_R;
            Temp = ecu.daq.Temp_LIM_Right;
            GPIO = GPIO_PIN_1;
            for(i=0;i<3;i++)
            {
                Thresh[i] = LIM_ptr[i];
            }

        }
        else if(arg2 == 1)
        {
            LIM_P = LIM_L;
            Temp = ecu.daq.Temp_LIM_Left;
            GPIO = GPIO_PIN_2;
            for(i=0;i<3;i++)
            {
                Thresh[i] = BMS_ptr[i];
            }

        }
    }

    else if(Reservoir_temp < Temp)
    {
        if(MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO) == GPIO)
        {

            if(Thresh[2]>=Temp)
            {
                PUMP_OFF(LIM_P);
            }
            else
            {
                return;
            }
        }
        if(MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO) == 0)
        {
            if(Temp>= Thresh[1])
            {
                PUMP_ON(LIM_P);
            }
            else
            {
                return;
            }
        }
    }
    else if(Reservoir_temp > Temp)
    {
        Bit[17] = true;
        if(MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO) == GPIO)
        {
            PUMP_OFF(LIM_P);
        }
        else
        {
            return;
        }
    }

}

//    if(arg1 == BMS_ptr)
//    {
//        if (arg2 == 0)
//            {
//                BMS_P = HVB_T;
//                Temp = ecu.daq.Temp_LIM_Right;
//                GPIO = GPIO_PIN_1;
//            }
//            if(arg2==1)
//            {
//                BMS_P = HVB_M
//                Temp = ecu.daq.Temp_LIM_Left;
//                GPIO = GPIO_PIN_2;
//            }
//            if(arg2==2)
//            {
//                BMS_P = HVB_B
//                Temp = ecu.daq.Temp_LIM_Left;
//                GPIO = GPIO_PIN_2;
//            }
//
//            if(MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO) == GPIO)
//            {
//                if(Temp>=LIM[0])
//                {
//
//                }
//                if(LIM[2]>=Temp)
//                {
//                    PUMP_OFF(LIM_R)
//                }
//                else
//                {
//                    return;
//                }
//            }
//            if(MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO) == 0)
//            {
//                if(Temp>= LIM[1])
//                {
//                    PUMP_ON(LIM_R)
//                }
//                else
//                {
//                    return;
//                }
//            }
//
//    }
//}


void PUMP_ON(PUMP Pump_name)
{
    switch(Pump_name)
    {
    case LIM_R:
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
        break;
    case LIM_L:
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);
        break;
    case HVB_T:
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);
        break;
    case HVB_M:
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4);
        break;
    case HVB_B:
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, GPIO_PIN_5);
        break;
    }

}

void PUMP_OFF(PUMP Pump_name)
{
    switch(Pump_name)
    {
    case LIM_R:
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,0);
        break;
    case LIM_L:
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
        break;
    case HVB_T:
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 0);
        break;
    case HVB_M:
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0);
        break;
    case HVB_B:
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, 0);
        break;
    }

}

void MCU_transmit(uint8_t data[3])
{
    sCANTxMessage[0].ui32MsgID = 0x113;
    sCANTxMessage[0].ui32MsgIDMask = 0 ;
    sCANTxMessage[0].ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANTxMessage[0].ui32MsgLen = 3;
    sCANTxMessage[0].pui8MsgData =data;

    MAP_CANMessageSet(CAN0_BASE,1,&sCANTxMessage[0], MSG_OBJ_TYPE_TX);
}

void Comparison(int arg1)
{
    if (arg1 == 0)
    {
        float HEMS_max = ecu.HEMS_thres.HEMS_Max;
        float H1F = 0.0f;
        float H2F = 0.0f;
        float H1B = 0.0f;
        float H2B = 0.0f;
        float HEMS_Temp[4];

       for(i=0;i<4;i++)
       {
           H1F += ecu.hems.Temp_HEMS_LF[i];
           H2B += ecu.hems.Temp_HEMS_RB[i];
           H2F += ecu.hems.Temp_HEMS_RF[i];
           H1B += ecu.hems.Temp_HEMS_LB[i];
       }
       HEMS_Temp[0] = H1F/4;
       HEMS_Temp[1] = H1B/4;
       HEMS_Temp[2] = H2F/4;
       HEMS_Temp[3] = H2B/4;

       if(HEMS_Temp[0]> HEMS_max)
       {
           Bit[0] = true;
       }
       else if(HEMS_Temp[1]>HEMS_max)
       {
           Bit[1] = true;
       }
       else if(HEMS_Temp[2]>HEMS_max)
       {
           Bit[2] = true;
       }
       else if(HEMS_Temp[3]>HEMS_max)
       {
           Bit[3] = true;
       }
       else
       {
           return;
       }
    }

    else if(arg1 == 1)
    {
        float LEMS_max = ecu.LEMS_thres.LEMS_Max;
        float L1F = 0.0f;
        float L2F = 0.0f;
        float L1B = 0.0f;
        float L2B = 0.0f;
        float LEMS_Temp[4];

       for(i=0;i<4;i++)
       {
           L1F += ecu.lems.Temp_LEMS_LF[i];
           L2B += ecu.lems.Temp_LEMS_RB[i];
           L2F += ecu.lems.Temp_LEMS_RF[i];
           L1B += ecu.lems.Temp_LEMS_LB[i];
       }
       LEMS_Temp[0] = L1F/4;
       LEMS_Temp[1] = L1B/4;
       LEMS_Temp[2] = L2F/4;
       LEMS_Temp[3] = L2B/4;

       if(LEMS_Temp[0]> LEMS_max)
       {
           Bit[4] = true;
       }
       else if(LEMS_Temp[1]>LEMS_max)
       {
           Bit[5] = true;
       }
       else if(LEMS_Temp[2]>LEMS_max)
       {
           Bit[6] = true;
       }
       else if(LEMS_Temp[3]>LEMS_max)
       {
           Bit[7] = true;
       }
       else
       {
           return;
       }
    }

    else if(arg1 == 2)
    {
        float Hbridge_max = ecu.hbrid_thres.H_Bridge_Max;
        float HB1F = 0.0f;
        float HB2F = 0.0f;
        float HB1B = 0.0f;
        float HB2B = 0.0f;
        float H_Bridge_Temp[4];

       for(i=0;i<4;i++)
       {
           HB1F += ecu.hbridge.Temp_HBridge_LF[i];
           HB2B += ecu.hbridge.Temp_HBridge_RB[i];
           HB2F += ecu.hbridge.Temp_HBridge_RF[i];
           HB1B += ecu.hbridge.Temp_HBridge_LB[i];
       }
       H_Bridge_Temp[0] = HB1F/4;
       H_Bridge_Temp[1] = HB1B/4;
       H_Bridge_Temp[2] = HB2F/4;
       H_Bridge_Temp[3] = HB2B/4;

       if(H_Bridge_Temp[0]> Hbridge_max)
       {
           Bit[8] = true;
       }
       else if(H_Bridge_Temp[1]> Hbridge_max)
       {
           Bit[9] = true;
       }
       else if(H_Bridge_Temp[2]> Hbridge_max)
       {
           Bit[10] = true;
       }
       else if(H_Bridge_Temp[3]> Hbridge_max)
       {
           Bit[11] = true;
       }
       else
       {
           return;
       }
    }

    else if(arg1 ==3)
    {
        float INV_max = ecu.PCU_thres.Inv_Max;
        float CONT_max = ecu.PCU_thres.Cont_Max;
        float PRE_max = ecu.PCU_thres.Pre_max;

        if(ecu.pcu.Temp_INV_Contact > CONT_max)
        {
            Bit[12] = true;
        }
        else if(ecu.pcu.Temp_INV_PreCh > PRE_max)
        {
            Bit[13] = true;
        }
        else if (ecu.pcu.Temp_INV_PCB > INV_max)
        {
            Bit[14] = true;
        }
        else
        {
            return;
        }
    }
    else if(arg1 == 4)
    {
        float PCM_temp;
        for (i=0;i<10;i++)
        {
            PCM_temp += (ecu.pcm.TEMP_PCM[i] / 10);
        }

        if(PCM_temp > ecu.PCM_thres.PCM_Max)
        {
            Bit[16]=true;

        }
        else
        {
            return;
        }
    }
}
float Heat_flow_rate(void)
{
    float T_out = ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_1a;
    float T_in[5] = {
        ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_1b,
        ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_2b,
        ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_3b,
        ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_4b,
        ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_5b
    };

    float Q1 = 0.0f;

    for (i = 0; i < 5; i++) {
        Q1 += T_in[i];
    }

    float Qdot = Mdot * Cp * (Q1 - 5*T_out);
    return Qdot*0.01;
}


void configureUART(void)
{
    /* Configure the UART and its pins.
     * This must be called before UARTprintf() */

    /* Enable the GPIO Peripheral used by the UART */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)))
    {
    }

    /* Enable UART2 */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

    /* Configure GPIO Pins for UART mode */
    MAP_GPIOPinConfigure(GPIO_PD4_U2RX);
    MAP_GPIOPinConfigure(GPIO_PD5_U2TX);
    MAP_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    /* Initialize the UART for console I/O */
    UARTStdioConfig(2, 115200, sysClock);
}

void configureCAN(void)
{
    /* Configure the CAN and its pins PA0 and PA1 @ 500Kbps */

    /* Enable the clock to the GPIO Port A and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)))
    {
    }

    /* Enable CAN0 */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    /* Configure GPIO Pins for CAN mode */
    MAP_GPIOPinConfigure(GPIO_PA0_CAN0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_CAN0TX);
    MAP_GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Initialize the CAN controller */
    MAP_CANInit(CAN0_BASE);

    /* Set up the bit rate for the CAN bus.  CAN bus is set to 500 Kbps */
    MAP_CANBitRateSet(CAN0_BASE, sysClock, 500000);

    /* Enable interrupts on the CAN peripheral */
    MAP_CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    /* Enable auto-retry on CAN transmit */
    MAP_CANRetrySet(CAN0_BASE, true);

    /* Enable the CAN interrupt */
    MAP_IntEnable(INT_CAN0);

    /* Enable the CAN for operation */
    MAP_CANEnable(CAN0_BASE);
}


void configureGPIOs(void)
{
    /* Enable the GPIO port that is used for the on-board LED */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    /* Check if the peripheral access is enabled */
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }

    /* Check if the peripheral access is enabled */
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    /* set GPIOs to default low. Set the direction as output,
     * and enable the GPIO pin for digital function */
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,
                     ~(GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5));
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

}

void configureSchedulerTimer(void)
{
    uint32_t timerPeriod;

    /* Enable Timer peripheral */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    /* Configure TIMER0 / TIMER_A as periodic timer */
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    /* 100Hz - 10ms */
    timerPeriod = (sysClock / 100);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, timerPeriod - 1);

    /* Enable interrupt for the timer timeout */
    MAP_IntEnable(INT_TIMER0A);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}

void receiveCANMsgManager(void)
{
    /* A new message is received */
    if (rxMsg)
    {
        /* Re-use the same message object that was used earlier to configure
         * the CAN */
        sCANRxMessage.pui8MsgData = (uint8_t *)&msgRxData;

        /* Read the message from the CAN */
        MAP_CANMessageGet(CAN0_BASE, 2, &sCANRxMessage, 0);

        /* Check the error flag to see if errors occurred */

        for(i=0; i < sCANRxMessage.ui32MsgLen; i++){
            msg2 = (msg2 << 8) + msgRxData[sCANRxMessage.ui32MsgLen -1 -i];
        }


        if (sCANRxMessage.ui32Flags & MSG_OBJ_DATA_LOST)
        {
            UARTprintf("\nCAN message loss detected\n");
        }
        else
        {
            parseCan(sCANRxMessage, msg2);
        }

        rxMsg = false;
    }
    else
    {
        if(errFlag)
        {
            UARTprintf("error while process the message\n");
            while(errFlag);
        }
    }
}

void CAN0_IRQHandler(void)
{
    uint32_t canStatus;

    /* Read the CAN interrupt status to find the cause of the interrupt */
    canStatus = MAP_CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    /* If the cause is a controller status interrupt, then get the status */
    if(canStatus == CAN_INT_INTID_STATUS)
    {
        /* Read the controller status.  This will return a field of status
         * error bits that can indicate various errors */
        canStatus = MAP_CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        /* Set a flag to indicate some errors may have occurred */
        errFlag = true;
    }

    /* Check if the cause is message object 1, which what we are using for
     * sending messages */
    else if(canStatus == 1)
    {
        /* Getting to this point means that the TX interrupt occurred on
         * message object 1, and the message TX is complete.  Clear the
         * message object interrupt */
        MAP_CANIntClear(CAN0_BASE, 1);

        /* Since the message was sent, clear any error flags */
        errFlag = false;
    }

    /* Check if the cause is message object 2, which what we are using for
     * sending messages */
    else if(canStatus == 2)
    {
        /* Getting to this point means that the TX interrupt occurred on
         * message object 2, and the message TX is complete.  Clear the
         * message object interrupt */
        MAP_CANIntClear(CAN0_BASE, 2);

        /* Since the message was sent, clear any error flags */
        errFlag = false;
    }

    /* Check if the cause is message object 3, which what we are using for
     * receiving messages */
    else if(canStatus == 3)
    {
        /* Getting to this point means that the RX interrupt occurred on
         * message object 3, and the message RX is complete.  Clear the
         * message object interrupt */
        MAP_CANIntClear(CAN0_BASE, 3);

        /* Set flag to indicate received message is pending */
        rxMsg = true;

        /* Since the message was sent, clear any error flags */
        errFlag = false;
    }
    else
    {
    }
}

void TIMER0A_IRQHandler(void)
{
    /* Clear the timer interrupt */
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Increment scheduler */
    schedulerTimer++;
}

