#pragma once

struct PCU_Thresholds
{
    float Inv_absMax;
    float Inv_Max;
    float Cont_absMax;
    float Cont_Max;
    float Pre_absMax;
    float Pre_max;
};


struct LIM_Thresholds
{
    float LIM_absMax;
    float LIM_Max;
    float LIM_Min;
};

struct BMS_Thresholds
{
    float BMS_absMax;
    float BMS_Max;
    float BMS_Min;
};

struct H_Bridge_Thresholds
{
    float H_Bridge_absMax;
    float H_Bridge_Max;
};

struct HEMS_Thresholds
{
    float HEMS_absMax;
    float HEMS_Max;
};

struct LEMS_Thresholds
{
    float LEMS_absMax;
    float LEMS_Max;

};

struct PCM_Thresholds
{
    float PCM_absMax;
    float PCM_Max;
};
struct IMU
{
    float IMU_Roll;
    float IMU_Pitch;


    struct IMU_Accel
    {
        float IMU_Acceleration_Lat;
        float IMU_Acceleration_Long;
        float IMU_Acceleration_Vert;

    } imu_accel;
};

struct PCM
{
    float TEMP_PCM[10];
};

struct ThCkt
{
    struct ThCkt_Pressure
    {
        float ThCkt_Pressure[5];

    } ThCkt_Pressure;

    struct ThCkt_Temp
    {
        float Temp_ThCkt_1a;
        float Temp_ThCkt_1b;
        float Temp_ThCkt_2a;
        float Temp_ThCkt_2b;
        float Temp_ThCkt_3a;

        float Temp_ThCkt_3b;
        float Temp_ThCkt_4a;
        float Temp_ThCkt_4b;
        float Temp_ThCkt_5a;
        float Temp_ThCkt_5b;

    } ThCkt_Temp;

};

struct HEMS
{
    float Temp_HEMS_LF[4];

    float Temp_HEMS_RF[4];

    float Temp_HEMS_LB[4];

    float Temp_HEMS_RB[4];
};

struct LEMS
{
    float Temp_LEMS_LF[4];

    float Temp_LEMS_LB[4];

    float Temp_LEMS_RF[4];

    float Temp_LEMS_RB[4];
};

struct HBridge
{
    float Temp_HBridge_LB[4];

    float Temp_HBridge_RB[4];

    float Temp_HBridge_LF[4];

    float Temp_HBridge_RF[4];
};

struct LCU
{
    float Airgap_LEMS_LF;
    float Airgap_HEMS_LF;
    float HBridge_OP_LF_HEMS;
    float LCU_PreCh_LF_1;
    float LCU_PreCh_LF_2;
    float HBridge_OP_LF_LEMS;

    float Airgap_LEMS_LB;
    float Airgap_HEMS_LB;
    float HBridge_OP_LB_HEMS;
    float LCU_PreCh_LB_1;
    float LCU_PreCh_LB_2;
    float HBridge_OP_LB_LEMS;

    float Airgap_HEMS_RF;
    float Airgap_LEMS_RF;
    float HBridge_OP_RF_HEMS;
    float LCU_PreCh_RF_1;
    float LCU_PreCh_RF_2;
    float HBridge_OP_RF_LEMS;

    float Airgap_HEMS_RB;
    float Airgap_LEMS_RB;
    float HBridge_OP_RB_HEMS;
    float LCU_PreCh_RB_1;
    float LCU_PreCh_RB_2;
    float HBridge_OP_RB_LEMS;
};

struct DAQ
{
    float DAQ_LIMAirGap_LB; //LB
    float Prox_LB_a;
    float Prox_LB_b;
    float resPressureLB;

    float DAQ_LIMAirGap_RF; //RF
    float Prox_RF_a;
    float Prox_RF_b;
    float resPressureRF;

    float Prox_RB_a; //RB
    float Prox_RB_b;
    float resPressureRB;
    float resPressureLIM_R;
    float Temp_LIM_Left;   
    
    float Prox_LF_a; //LF
    float Prox_LF_b;
    float resPressureLF;
    float resPressureLIM_L;
    float Temp_LIM_Right;
};

struct PCU
{
    float INV_PreCh_Bus_Volt;
    float LIM_Curr;
    float Pod_Position;
    float Pod_Velocity;
    float INV_Contact_Curr;
    float INV_PreCh_Curr;
        // COMING FROM DAQ_LF AND DAQ_RB.    ALWAYS REMEMBER THAT " CANDB IS GOD !" ALWAYS REFER THAT FOR CLARITY.

    float Temp_INV_Contact;
    float Temp_INV_PCB;
    float Temp_INV_PreCh;
};

struct ECU
{
    struct IMU imu;
    struct PCM pcm;
    struct ThCkt ThCkt;
    struct HEMS hems;
    struct LEMS lems;
    struct HBridge hbridge;
    struct LCU lcu;
    struct DAQ daq;
    struct PCU pcu;
    struct LIM_Thresholds lim_thres;
    struct BMS_Thresholds bms_thres;
    struct H_Bridge_Thresholds hbrid_thres;
    struct HEMS_Thresholds HEMS_thres;
    struct LEMS_Thresholds LEMS_thres;
    struct PCU_Thresholds PCU_thres;
    struct PCM_Thresholds PCM_thres;

} ecu;




















































































