#include "CAN_h.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

#include "ti/devices/msp432e4/driverlib/driverlib.h"

/* Driver Header files */
#include <ti/drivers/CAN.h>
#include <ti/drivers/GPIO.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
// #include <cstdint>

#define IMU_FACTOR 32768
#define TEMP_Factor 10
#define Prox_Factor 1
#define ThCkt_Pressure_Factor 200
#define IMU_Accel_Factor 100
#define Airgap_Factor 100
#define HBridge_Output_Factor 100
#define LCU_PreCh_Factor 100
#define DAQ_LIMAirGap_Factor 100
#define resPressure_Factor 100/3
#define INV_PreCh_Bus_Volt_Factor 1
#define LIM_Curr_Factor 10
#define INV_PreCh_Bus_Volt_Factor 1
#define INV_PreCh_Bus_Volt_Factor 1
#define Pod_Position_Factor 10
#define Pod_Velocity_Factor 10
#define INV_Contact_Curr_Factor 1
#define INV_PreCh_Curr_Factor 1

#define IMU_OFFSET 250
#define IMU_Accel_Offset 320
#define TEMP_PCM_Offset 35
#define TEMP_Offset 0
#define TEMP_HBridge_Offset 20
#define TEMP_ThCkt_Offset 35
#define resPressure_Offset 0
#define Prox_Offset 0
#define Airgap_Offset 0
#define HBridge_OP_Offset 0
#define LCU_PreCh_Offset 0
#define INV_PreCh_Bus_Volt_Offset 0
#define LIM_Curr_Offset 0
#define Pod_Position_Offset 0
#define Pod_Velocity_Offset 0
#define INV_Contact_Curr_Offset 0
#define INV_PreCh_Curr_Offset 0


uint64_t msg = 0;

/* Function to parse data received from Bus */

float parseData(uint16_t offset, uint16_t factor, int len, int s_bit) {

    uint64_t buffer = 0;
    float parsed_msg = 0;

    buffer = msg << (64 - (s_bit + len));
    buffer = buffer >> (64 - len);
    parsed_msg = ((float)buffer / factor) - offset;

    return parsed_msg;
}

int parseCan(tCANMsgObject *frame , uint64_t msg1)
{

    tCANMsgObject Bus_canFrame = *(frame);

    // Event-driven messages

    // if (Bus_canFrame.id >= 0x2E && Bus_canFrame.id < 0x2EE) {
    //     return Bus_canFrame.id;
    // } 


    //initial receiving
    int i;

    msg = msg1;


    //switch case
    switch (Bus_canFrame.ui32MsgID){

        case 0x360:

        // IMU Roll and Pitch

            // 1st signal
            ecu.imu.IMU_Roll = parseData(IMU_OFFSET, IMU_FACTOR, 24, 0);

            // 2nd signal
            ecu.imu.IMU_Pitch = parseData(IMU_OFFSET, IMU_FACTOR, 24, 24);

            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x252:

        // IMU Acceleration - Lat, Long & Vert

            // 1st signal
            ecu.imu.imu_accel.IMU_Acceleration_Lat = parseData(IMU_Accel_Offset, IMU_Accel_Factor, 16, 0);

            // 2nd signal
            ecu.imu.imu_accel.IMU_Acceleration_Long = parseData(IMU_Accel_Offset, IMU_Accel_Factor, 16, 16);

            //3rd signal
            ecu.imu.imu_accel.IMU_Acceleration_Vert = parseData(IMU_Accel_Offset, IMU_Accel_Factor, 16, 32);

            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x310:

        // Temperature - PCM 1

            // 1st signal
            ecu.pcm.TEMP_PCM[0] = parseData(TEMP_PCM_Offset, TEMP_Factor, 11, 0);

            // 2nd signal
            ecu.pcm.TEMP_PCM[1] = parseData(TEMP_PCM_Offset, TEMP_Factor, 11, 11);

            //3rd signal
            ecu.pcm.TEMP_PCM[2] = parseData(TEMP_PCM_Offset, TEMP_Factor, 11, 22);


            //4th signal
            ecu.pcm.TEMP_PCM[3] = parseData(TEMP_PCM_Offset, TEMP_Factor, 11, 33);

            //5th signal
            ecu.pcm.TEMP_PCM[4] = parseData(TEMP_PCM_Offset, TEMP_Factor, 11, 44);

            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x311:

        // Temperature - PCM 2

            // 1st signal
            ecu.pcm.TEMP_PCM[5] = parseData(TEMP_PCM_Offset, TEMP_Factor, 11, 0);

            // 2nd signal
            ecu.pcm.TEMP_PCM[6] = parseData(TEMP_PCM_Offset, TEMP_Factor, 11, 11);

            //3rd signal
            ecu.pcm.TEMP_PCM[7] = parseData(TEMP_PCM_Offset, TEMP_Factor, 11, 22);


            //4th signal
            ecu.pcm.TEMP_PCM[8] = parseData(TEMP_PCM_Offset, TEMP_Factor, 11, 33);

            //5th signal
            ecu.pcm.TEMP_PCM[9] = parseData(TEMP_PCM_Offset, TEMP_Factor, 11, 44);

            break;

        //-----------------------------------------------------------------------------------------------------------//
        case 0x4E2:

        // Thermal Circuit - Pressure

            // 1st signal
            ecu.ThCkt.ThCkt_Pressure.ThCkt_Pressure[0] = parseData(TEMP_Offset, ThCkt_Pressure_Factor, 11, 0);

            // 2nd signal
            ecu.ThCkt.ThCkt_Pressure.ThCkt_Pressure[1] = parseData(TEMP_Offset, ThCkt_Pressure_Factor, 11, 11);

            //3rd signal
            ecu.ThCkt.ThCkt_Pressure.ThCkt_Pressure[2] = parseData(TEMP_Offset, ThCkt_Pressure_Factor, 11, 22);

            //4th signal
            ecu.ThCkt.ThCkt_Pressure.ThCkt_Pressure[3] = parseData(TEMP_Offset, ThCkt_Pressure_Factor, 11, 33);

            //5th signal
            ecu.ThCkt.ThCkt_Pressure.ThCkt_Pressure[4] = parseData(TEMP_Offset, ThCkt_Pressure_Factor, 11, 44);

            break;
        //-----------------------------------------------------------------------------------------------------------//

        case 0x532:

        // Temperature - HEMS - Left Forward

            // 1st signal
            ecu.hems.Temp_HEMS_LF[0] = parseData(TEMP_Offset, TEMP_Factor, 10, 0);

            // 2nd signal
            ecu.hems.Temp_HEMS_LF[1] = parseData(TEMP_Offset, TEMP_Factor, 10, 10);

            //3rd signal
            ecu.hems.Temp_HEMS_LF[2] = parseData(TEMP_Offset, TEMP_Factor, 10, 20);

            //4th signal
            ecu.hems.Temp_HEMS_LF[3] = parseData(TEMP_Offset, TEMP_Factor, 10, 30);

            break;

        //-----------------------------------------------------------------------------------------------------------//
        case 0x53C:

        // Temperature - HEMS - Right Forward

            // 1st signal
            ecu.hems.Temp_HEMS_RF[0] = parseData(TEMP_Offset, TEMP_Factor, 10, 0);

            // 2nd signal
            ecu.hems.Temp_HEMS_RF[1] = parseData(TEMP_Offset, TEMP_Factor, 10, 10);

            //3rd signal
            ecu.hems.Temp_HEMS_RF[2] = parseData(TEMP_Offset, TEMP_Factor, 10, 20);

            //4th signal
            ecu.hems.Temp_HEMS_RF[3] = parseData(TEMP_Offset, TEMP_Factor, 10, 30);

            break;

        //-----------------------------------------------------------------------------------------------------------//
        case 0x528:

        // Temperature - HEMS - Left Back

            // 1st signal
            ecu.hems.Temp_HEMS_LB[0] = parseData(TEMP_Offset, TEMP_Factor, 10, 0);

            // 2nd signal
            ecu.hems.Temp_HEMS_LB[1] = parseData(TEMP_Offset, TEMP_Factor, 10, 10);

            //3rd signal
            ecu.hems.Temp_HEMS_LB[2] = parseData(TEMP_Offset, TEMP_Factor, 10, 20);

            //4th signal
            ecu.hems.Temp_HEMS_LB[3] = parseData(TEMP_Offset, TEMP_Factor, 10, 30);

            break;

        //-----------------------------------------------------------------------------------------------------------//
        case 0x51E:

        // Temperature - HEMS - Right Back

            // 1st signal
            ecu.hems.Temp_HEMS_RB[0] = parseData(TEMP_Offset, TEMP_Factor, 10, 0);

            // 2nd signal
            ecu.hems.Temp_HEMS_RB[1] = parseData(TEMP_Offset, TEMP_Factor, 10, 10);

            //3rd signal
            ecu.hems.Temp_HEMS_RB[2] = parseData(TEMP_Offset, TEMP_Factor, 10, 20);

            //4th signal
            ecu.hems.Temp_HEMS_RB[3] = parseData(TEMP_Offset, TEMP_Factor, 10, 30);

            break;

        //------------------------------------------------------------------------------------------------------------//
        case 0x550:

        // Temperature - LEMS - Left Forward

            // 1st signal
            ecu.lems.Temp_LEMS_LF[0] = parseData(TEMP_Offset, TEMP_Factor, 10, 0);

            // 2nd signal
            ecu.lems.Temp_LEMS_LF[1] = parseData(TEMP_Offset, TEMP_Factor, 10, 10);

            //3rd signal
            ecu.lems.Temp_LEMS_LF[2] = parseData(TEMP_Offset, TEMP_Factor, 10, 20);

            //4th signal
            ecu.lems.Temp_LEMS_LF[3] = parseData(TEMP_Offset, TEMP_Factor, 10, 30);

            break;

        //-----------------------------------------------------------------------------------------------------------//



        case 0x546:

        // Temperature - LEMS - Left Back

            // 1st signal
            ecu.lems.Temp_LEMS_LB[0] = parseData(TEMP_Offset, TEMP_Factor, 10, 0);

            // 2nd signal
            ecu.lems.Temp_LEMS_LB[1] = parseData(TEMP_Offset, TEMP_Factor, 10, 10);

            //3rd signal
            ecu.lems.Temp_LEMS_LB[2] = parseData(TEMP_Offset, TEMP_Factor, 10, 20);

            //4th signal
            ecu.lems.Temp_LEMS_LB[3] = parseData(TEMP_Offset, TEMP_Factor, 10, 30);

            break;

        //-----------------------------------------------------------------------------------------------------------//

        case 0x564:

        // Temperature - LEMS - Right Forward

            // 1st signal
            ecu.lems.Temp_LEMS_RF[0] = parseData(TEMP_Offset, TEMP_Factor, 10, 0);

            // 2nd signal
            ecu.lems.Temp_LEMS_RF[1] = parseData(TEMP_Offset, TEMP_Factor, 10, 10);

            //3rd signal
            ecu.lems.Temp_LEMS_RF[2] = parseData(TEMP_Offset, TEMP_Factor, 10, 20);

            //4th signal
            ecu.lems.Temp_LEMS_RF[3] = parseData(TEMP_Offset, TEMP_Factor, 10, 30);

            break;

        //-----------------------------------------------------------------------------------------------------------//

        case 0x55A:

        // Temperature - LEMS - Right Back

            // 1st signal
            ecu.lems.Temp_LEMS_RB[0] = parseData(TEMP_Offset, TEMP_Factor, 10, 0);

            // 2nd signal
            ecu.lems.Temp_LEMS_RB[1] = parseData(TEMP_Offset, TEMP_Factor, 10, 10);

            //3rd signal
            ecu.lems.Temp_LEMS_RB[2] = parseData(TEMP_Offset, TEMP_Factor, 10, 20);

            //4th signal
            ecu.lems.Temp_LEMS_RB[3] = parseData(TEMP_Offset, TEMP_Factor, 10, 30);

            break;

        //-----------------------------------------------------------------------------------------------------------//

        case 0x314:

        //  Temperature - HBridge - Left Back

            // 1st signal
            ecu.hbridge.Temp_HBridge_LB[0] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 0);

            // 2nd signal
            ecu.hbridge.Temp_HBridge_LB[1] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 11);

            //3rd signal
            ecu.hbridge.Temp_HBridge_LB[2] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 22);

            //4th signal
            ecu.hbridge.Temp_HBridge_LB[3] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 33);

            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x500:

        //  Temperature - HBridge - Right Back

            // 1st signal
            ecu.hbridge.Temp_HBridge_RB[0] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 0);

            // 2nd signal
            ecu.hbridge.Temp_HBridge_RB[1] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 11);

            //3rd signal
            ecu.hbridge.Temp_HBridge_RB[2] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 22);

            //4th signal
            ecu.hbridge.Temp_HBridge_RB[3] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 33);

            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x316:

        //  Temperature - HBridge - Left Forward

            // 1st signal
            ecu.hbridge.Temp_HBridge_LF[0] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 0);

            // 2nd signal
            ecu.hbridge.Temp_HBridge_LF[1] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 11);

            //3rd signal
            ecu.hbridge.Temp_HBridge_LF[2] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 22);

            //4th signal
            ecu.hbridge.Temp_HBridge_LF[3] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 33);

            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x4F6:

        //  Temperature - HBridge - Right Forward

            // 1st signal
            ecu.hbridge.Temp_HBridge_RF[0] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 0);

            // 2nd signal
            ecu.hbridge.Temp_HBridge_RF[1] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 11);

            //3rd signal
            ecu.hbridge.Temp_HBridge_RF[2] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 22);

            //4th signal
            ecu.hbridge.Temp_HBridge_RF[3] = parseData(TEMP_HBridge_Offset, TEMP_Factor, 11, 33);

            break;
        // ----------------------------------------------------------------------------------------------------------//

        case 0x292:

        // Levitation Control Unit - Left Forward

            // 1st Signal - Airgap - LEMS Left Forward
            ecu.lcu.Airgap_LEMS_LF = parseData(Airgap_Offset, Airgap_Factor, 12, 0);

            // 2nd Signal - Airgap - HEMS Left Forward
            ecu.lcu.Airgap_HEMS_LF = parseData(Airgap_Offset, Airgap_Factor, 12, 12);

            // 3rd Signal - HBridge - Output - Left Forward
//            ecu.lcu.HBridge_OP_LF = parseData(HBridge_OP_Offset, HBridge_Output_Factor, 10, 24);

            // 4th Signal - Levitation Control Unit - PreCharge - Left Forward - 1
            ecu.lcu.LCU_PreCh_LF_1 = parseData(LCU_PreCh_Offset, LCU_PreCh_Factor, 10, 34);

            // 5th Signal - Levitation Control Unit - PreCharge - Left Forward - 2
            ecu.lcu.LCU_PreCh_LF_2 = parseData(LCU_PreCh_Offset, LCU_PreCh_Factor, 10, 44);

            //6th Signal - HBridge - Output - LEMS - left Forward
            ecu.lcu.HBridge_OP_LF_LEMS = parseData(HBridge_OP_Offset, HBridge_Output_Factor, 10, 54);


            break;
        // ----------------------------------------------------------------------------------------------------------//

        case 0x390:

            // Levitation Control Unit - Left Back

                // 1st Signal - Airgap - LEMS Left Back
            ecu.lcu.Airgap_LEMS_LB = parseData(Airgap_Offset, Airgap_Factor, 12, 0);

            // 2nd Signal - Airgap - HEMS Left Back
            ecu.lcu.Airgap_HEMS_LB = parseData(Airgap_Offset, Airgap_Factor, 12, 12);

            // 3rd Signal - HBridge - Output -HEMS - Left Back
            ecu.lcu.HBridge_OP_LB_HEMS = parseData(HBridge_OP_Offset, HBridge_Output_Factor, 10, 24);

            // 4th Signal - Levitation Control Unit - PreCharge - Left Back - 1
            ecu.lcu.LCU_PreCh_LB_1 = parseData(LCU_PreCh_Offset, LCU_PreCh_Factor, 10, 34);

            // 5th Signal - Levitation Control Unit - PreCharge - Left Back - 2
            ecu.lcu.LCU_PreCh_LB_2 = parseData(LCU_PreCh_Offset, LCU_PreCh_Factor, 10, 44);

            //6th Signal - HBridge - Output - LEMS - LEft back
            ecu.lcu.HBridge_OP_LB_LEMS = parseData(HBridge_OP_Offset, HBridge_Output_Factor, 10, 54);
            break;
            // ----------------------------------------------------------------------------------------------------------//

        case 0x3A0:

        // Levitation Control Unit - Right Forward

            // 1st Signal - Airgap - LEMS Right Forward
            ecu.lcu.Airgap_LEMS_RF = parseData(Airgap_Offset, Airgap_Factor, 12, 0);

            // 2nd Signal - Airgap - HEMS Right Forward
            ecu.lcu.Airgap_HEMS_RF = parseData(Airgap_Offset, Airgap_Factor, 12, 12);

            // 3rd Signal - HBridge - Output -HEMS - Right Forward
            ecu.lcu.HBridge_OP_RF_HEMS = parseData(HBridge_OP_Offset, HBridge_Output_Factor, 10, 24);

            // 4th Signal - Levitation Control Unit - PreCharge - Right Forward - 1
            ecu.lcu.LCU_PreCh_RF_1 = parseData(LCU_PreCh_Offset, LCU_PreCh_Factor, 10, 34);

            // 5th Signal - Levitation Control Unit - PreCharge - Right Forward - 2
            ecu.lcu.LCU_PreCh_RF_2 = parseData(LCU_PreCh_Offset, LCU_PreCh_Factor, 10, 44);
            
            //6th Signal - HBridge - Output - LEMS - Right Forward
            ecu.lcu.HBridge_OP_RF_LEMS = parseData(HBridge_OP_Offset, HBridge_Output_Factor, 10, 54);
            break;
        // ----------------------------------------------------------------------------------------------------------//

        case 0x3B0:

        // Levitation Control Unit - Right Back

            // 1st Signal - Airgap - HEMS Right Back
            ecu.lcu.Airgap_HEMS_RB = parseData(Airgap_Offset, Airgap_Factor, 12, 0);

            // 2nd Signal - Airgap - LEMS Right Back
            ecu.lcu.Airgap_LEMS_RB = parseData(Airgap_Offset, Airgap_Factor, 12, 12);

            // 3rd Signal - HBridge - Output - Right Back
//            ecu.lcu.HBridge_OP_RB = parseData(HBridge_OP_Offset, HBridge_Output_Factor, 10, 24);

            // 4th Signal - Levitation Control Unit - PreCharge - Right Back - 1
            ecu.lcu.LCU_PreCh_RB_1 = parseData(LCU_PreCh_Offset, LCU_PreCh_Factor, 10, 34);

            // 5th Signal - Levitation Control Unit - PreCharge - Right Back - 2
            ecu.lcu.LCU_PreCh_RB_2 = parseData(LCU_PreCh_Offset, LCU_PreCh_Factor, 10, 44);

            //6th Signal - HBridge - Output - LEMS - Right back
            ecu.lcu.HBridge_OP_RB_LEMS = parseData(HBridge_OP_Offset, HBridge_Output_Factor, 10, 54);
            break;
        // ----------------------------------------------------------------------------------------------------------//

        case 0x319:

        // Temperature - Thermal Circuit - 1a3a

            // 1st signal
            ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_1a = parseData(TEMP_ThCkt_Offset, TEMP_Factor, 11, 0);

            // 2nd signal
            ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_1b = parseData(TEMP_ThCkt_Offset, TEMP_Factor, 11, 11);

            //3rd signal
            ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_2a = parseData(TEMP_ThCkt_Offset, TEMP_Factor, 11, 22);

            //4th signal
            ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_2b = parseData(TEMP_ThCkt_Offset, TEMP_Factor, 11, 33);

            //5th signal
            ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_3a = parseData(TEMP_ThCkt_Offset, TEMP_Factor, 11, 44);

            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x320:

        // Temperature - Thermal Circuit - 3b5b

            // 1st signal
            ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_3b = parseData(TEMP_ThCkt_Offset, TEMP_Factor, 11, 0);

            // 2nd signal
            ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_4a = parseData(TEMP_ThCkt_Offset, TEMP_Factor, 11, 11);

            //3rd signal
            ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_4b = parseData(TEMP_ThCkt_Offset, TEMP_Factor, 11, 22);


            //4th signal
            ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_5a = parseData(TEMP_ThCkt_Offset, TEMP_Factor, 11, 33);

            //5th signal
            ecu.ThCkt.ThCkt_Temp.Temp_ThCkt_5b = parseData(TEMP_ThCkt_Offset, TEMP_Factor, 11, 44);

            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x350:

        // DAQ - Left Back

            // 1st Signal - Linear Induction Motor - AirGap
            ecu.daq.DAQ_LIMAirGap_LB = parseData(0, DAQ_LIMAirGap_Factor, 12, 0);

            // 2nd Signal - Prox - LBa
            ecu.daq.Prox_LB_a = parseData(Prox_Offset, Prox_Factor, 1, 12);

            // 3rd Signal - Prox - LBb
            ecu.daq.Prox_LB_b = parseData(Prox_Offset, Prox_Factor, 1, 13);

            // 4th Signal - Reservoir Pressure - LB
            ecu.daq.resPressureLB = parseData(resPressure_Offset, resPressure_Factor, 8, 16);

            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x340:

        // DAQ - Right Forward

            // 1st Signal - Linear Induction Motor - AirGap
            ecu.daq.DAQ_LIMAirGap_RF = parseData(0, DAQ_LIMAirGap_Factor, 12, 0);

            // 2nd Signal - Prox - RFa
            ecu.daq.Prox_RF_a = parseData(Prox_Offset, Prox_Factor, 1, 12);

            // 3rd Signal - Prox - RFb
            ecu.daq.Prox_RF_b = parseData(Prox_Offset, Prox_Factor, 1, 13);

            // 4th Signal - Reservoir Pressure - RF
            ecu.daq.resPressureRF = parseData(resPressure_Offset, resPressure_Factor, 8, 16);

            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x3D0:

        // DAQ - Right Back

            // 1st Signal - Prox - RBa
            ecu.daq.Prox_RB_a = parseData(Prox_Offset, Prox_Factor, 1, 0);

            // 2nd Signal - Prox - RBb
            ecu.daq.Prox_RB_a = parseData(Prox_Offset, Prox_Factor, 1, 1);

            // 3rd Signal - Reservoir Pressure - RB
            ecu.daq.resPressureRB = parseData(resPressure_Offset, resPressure_Factor, 8, 8);

            // 4th Signal - Reservoir Pressure - RB
            ecu.daq.resPressureLIM_R = parseData(resPressure_Offset, resPressure_Factor, 8, 16);

            //5th Signal - LIM Temperature Right
            ecu.daq.Temp_LIM_Right = parseData(TEMP_Offset, TEMP_Factor, 10, 24);

            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x3C0:

        // DAQ - Left Forward <---How about uint8_t?-->

            // 1st Signal - Prox - RBa
            ecu.daq.Prox_LF_a = parseData(Prox_Offset, Prox_Factor, 1, 0);

            // 2nd Signal - Prox - RBb
            ecu.daq.Prox_LF_b = parseData(Prox_Offset, Prox_Factor, 1, 1);
            // 3rd Signal - Reservoir Pressure - RB

            ecu.daq.resPressureLF = parseData(resPressure_Offset, resPressure_Factor, 8, 8);

            // 4th Signal - Reservoir Pressure - RB
            ecu.daq.resPressureLIM_L = parseData(resPressure_Offset, resPressure_Factor, 8, 16);

            //5th Signal - LIM Temperature Right
            ecu.daq.Temp_LIM_Left = parseData(TEMP_Offset, TEMP_Factor, 10, 24);
            break;

        // ----------------------------------------------------------------------------------------------------------//

        case 0x330:

            // PCU - LIM

            // 1st Signal
            ecu.pcu.INV_PreCh_Bus_Volt = parseData(INV_PreCh_Bus_Volt_Offset, INV_PreCh_Bus_Volt_Factor, 9, 0);

            // 2nd Signal
            ecu.pcu.LIM_Curr = parseData(LIM_Curr_Offset, LIM_Curr_Factor, 11, 9);

            //  3rd Signal
            ecu.pcu.Pod_Position = parseData(Pod_Position_Offset, Pod_Position_Factor, 9, 20);

            // 4th Signal
            ecu.pcu.Pod_Velocity = parseData(Pod_Velocity_Offset, Pod_Velocity_Factor, 9, 29);

            // 5th Signal
            ecu.pcu.INV_Contact_Curr = parseData(INV_Contact_Curr_Offset, INV_Contact_Curr_Factor, 11, 38);

            // 6th Signal
            ecu.pcu.INV_PreCh_Curr = parseData(INV_PreCh_Curr_Offset, INV_PreCh_Curr_Factor, 11, 49);

        // ----------------------------------------------------------------------------------------------------------//

        case 0x4EC:

        // PCU - TEMP

        // 1st Signal
        ecu.pcu.Temp_INV_Contact = parseData(TEMP_Offset, TEMP_Factor, 10, 0);

        // 2nd Signal
        ecu.pcu.Temp_INV_PCB = parseData(TEMP_Offset, TEMP_Factor, 10, 10);

        // 3rd Signal
        ecu.pcu.Temp_INV_PreCh = parseData(TEMP_Offset, TEMP_Factor, 10, 20);
    }

    return -1;
}
