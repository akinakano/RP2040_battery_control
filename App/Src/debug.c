#include <stdio.h>

#include "debug.h"
#include "console.h"
#include "comm_mpsv.h"
#include "motion_controller.h"
#include "imu_common.h"
#include "imu_icm42688.h"
#include "comm_apmp.h"
#include "comm_battery.h"
#include "tim.h"
#include "gpio.h"
#include "vqfInstance.h"

extern uint8_t imu_debug_flag;
extern struct BatteryStatus_st BatteryStatus;
extern int Status_15V;

void debug_command(int c);
void debug_print(void);

void debug_menu(void){
    printf("\n");
    printf("***** PURIMOKUN MOTION CONTROL ver.20%02d%02d%02d.%02d *****\n"
        ,MP_VERSION_YY, MP_VERSION_MM, MP_VERSION_DD, MP_VERSION_NN);
    printf("Debug Command \n");
    printf("p : print log \n");
    printf("b : battery info \n");
    printf("z : break \n");

    int c = getchar();
    printf("%c\n",c);
    if(c == 'p'){
      while(1){
          debug_print();
          c = checkchar();

          if ( c== 'z') {
            break;
          }

            debug_command(c);
      }
    }
    if( c == 'b') {
      printf("\n-----Battery Info-----\n");
      printf("Current    : %dmA\n", (int)BatteryStatus.Current);
      printf("Voltage    : %dmV\n", BatteryStatus.Voltage);
      printf("Capacity   : %d%%\n", BatteryStatus.Capacity);
      printf("Temperture : %d.%d degree\n", (BatteryStatus.Temperture - 2732) / 10, (BatteryStatus.Temperture - 2732) % 10);
      printf("Serial     : %04d-%02d-%02d-%05d\n", (BatteryStatus.ManufactureDate >> 9) + 1980, (BatteryStatus.ManufactureDate >> 5) & 0x0f, BatteryStatus.ManufactureDate & 0x1f, BatteryStatus.SerialNumber);
      printf("CFET       : %s\n", BatteryStatus.ManufacturerAccess & (1 << 3) ? "on" : "off");
      printf("ValidFlag  : %d\n", BatteryStatus.ValidFlag);
      printf("Power15V   : %d\n", Power15V);
      printf("----------------------\n");
    }
    if( c == '1') {
      GPIO_15V(1);
    }
    if( c == '0') {
      GPIO_15V(0);
    }
    //apmp_data.cmd = AP_MP_CMD_IDLE;
}

void debug_print(void){
    printf("st %1u %6u %1u %1d ", MC_state.state, MC_state.time_count , state_mp_to_ap, apmp_data.cmd);

    // MP制禦系の表示
#if 1
    printf("vraw %6d %6d ", (int)(BVC.vel_cmd_raw[X_AXIS] * 1000.0f), (int)(BVC.vel_cmd_raw[Z_AXIS] * 1800.0f / SMC_PI_F));
    //printf("vcmd %6d %6d ", (int)(BVC.vel_cmd[X_AXIS] * 1000.0f), (int)(BVC.vel_cmd[Z_AXIS] * 1800.0f / SMC_PI_F));
    //printf("vref %6d %6d ", (int)(BVC.vel_ref[X_AXIS] * 1000.0f), (int)(BVC.vel_ref[Z_AXIS] * 1800.0f / SMC_PI_F));
    //printf("vres %6d %6d ", (int)(BVC.vel_res[X_AXIS] * 1000.0f), (int)(BVC.vel_res[Z_AXIS] * 1800.0f / SMC_PI_F));
#endif

#if 0
    // MP内の位置、オドメトリの表示
    printf("pcmd %6d %6d %6d "
        , (int)(BVC.pos_cmd[X_AXIS] * 1000.0f)
        , (int)(BVC.pos_cmd[Y_AXIS] * 1000.0f)
        , (int)(BVC.pos_cmd[Z_AXIS] * 1800.0f / SMC_PI_F));
    printf("pres %6d %6d %6d "
        , (int32_t)(BVC.pos_res[X_AXIS] * 1000.0f)
        , (int)(BVC.pos_res[Y_AXIS] * 1000.0f)
        , (int)(BVC.pos_res[Z_AXIS] * 1800.0f / SMC_PI_F)
        );

    printf("bias %4d %6d %6d "
        ,imu_data.calib_count
        ,(int)(imu_data.gyro[Z_AXIS] * 180.0f * 1000.0f / SMC_PI_F)
        ,(int)(imu_data.gyro_bias[Z_AXIS] * 180.0f * 1000.0f / SMC_PI_F)
        //,(int)(imu_data.theta[Z_AXIS] * 180.0f * 1000.0f / SMC_PI_F)
    );
#endif

#if 0
    printf("mpap %02x%02x%02x%02x "
        ,mpap_buff[16]
        ,mpap_buff[17]
        ,mpap_buff[18]
        ,mpap_buff[19]);
#endif

#if 0
    printf("odom %6d %6d %6d %6d "
        //, (int)(ODOM_data.sv1_delta_len * 1000.0f * MOTION_CONTROL_FREQ)
        //, (int)(ODOM_data.sv2_delta_len * 1000.0f * MOTION_CONTROL_FREQ)
        //, (int)(ODOM_data.delta_theta_gyro * 180.0f / SMC_PI_F * MOTION_CONTROL_FREQ)
        //, (int)(ODOM_data.delta_theta_odom * 180.0f / SMC_PI_F * MOTION_CONTROL_FREQ)
        //, (int)(fabsf(ODOM_data.delta_theta_gyro - ODOM_data.delta_theta_odom) * 180.0f / SMC_PI_F * MOTION_CONTROL_FREQ)
        , (int)(ODOM_data.delta_theta * 180.0f / SMC_PI_F * MOTION_CONTROL_FREQ)
        , (int)(ODOM_data.delta_len * 1000.0f * MOTION_CONTROL_FREQ)
        //, (int)(ODOM_data.x_error_body * 1000.0f)
        //, (int)(ODOM_data.y_error_body * 1000.0f)
        , (int)(ODOM_data.SV1_force * 1000.0f)
        , (int)(ODOM_data.SV2_force * 1000.0f)
    );
#endif

    // imu系の表示
#if 1
    printf("acc %6d %6d %6d "
        , (int)(imu_data.acc[X_AXIS] * 1000.0f)
        , (int)(imu_data.acc[Y_AXIS] * 1000.0f)
        , (int)(imu_data.acc[Z_AXIS] * 1000.0f));
    printf("gyro %6d %6d %6d "
        , (int)(imu_data.gyro[X_AXIS] * 1800.0f / SMC_PI_F)
        , (int)(imu_data.gyro[Y_AXIS] * 1800.0f / SMC_PI_F)
        , (int)(imu_data.gyro[Z_AXIS] * 1800.0f / SMC_PI_F));

    printf("gbias %6d "
        , (int)(imu_data.gyro_bias[Z_AXIS] * 180.0f * 1000.0f / SMC_PI_F));
#endif

#if 0 // -----VQF
    // VQF
    float bias[3];
    vqf_GetBiasEstimate(bias);
    printf("vbias %6d "
        ,(int)(bias[2] * 180.0f * 1000.0f / SMC_PI_F));

    float quat[4];
    vqf_GetQuat6D(quat);
    printf("VQF %d %6d %6d %6d %6d %6d %6d "
        ,vqf_GetRestDetected()
        ,(int)(quat[0] * 1000.0f)
        ,(int)(quat[1] * 10000.0f)
        ,(int)(quat[2] * 10000.0f)
        ,(int)(quat[3] * 10000.0f)
        ,vqf_GetDebugCountThGyro()
        ,vqf_GetDebugCountThAcc());

    float euler[3];
    vqf_QuatToEuler(euler, quat);
    printf("euler %6d %6d %6d "
        ,(int)(euler[0] * 18000.0f / SMC_PI_F)
        ,(int)(euler[1] * 18000.0f / SMC_PI_F)
        ,(int)(euler[2] * 18000.0f / SMC_PI_F));
#endif

#if 0
    // SV系の表示
    printf("SV %1d %1d ", sv_data[0].current_state, sv_data[1].current_state);
    //COM_MP_TO_SV * cmd_mp_sv = get_send_cmd_handle();
    //printf("force %6d %6d ", cmd_mp_sv->force[SV1_FR], cmd_mp_sv->force[SV2_FL]);

    //printf("cnt %6d %6d ", sv_data[0].cnt,sv_data[1].cnt);
    //printf("mr %6d %6d ", sv_data[SV1_FR].mr, sv_data[SV2_FL].mr);
    //printf("hall %6d %6d %6d %6d ", sv_data[SV1_FR].hall, sv_data[SV2_FL].hall, sv_data[SV3_HR].hall, sv_data[SV4_HL].hall);

    //printf("th %6d %6d %6d %6d ", (int)(sv_res[SV1_FR].th_res * 180.0f / 3.14f), (int)(sv_res[SV2_FL].th_res * 180.0f / 3.14f), (int)(sv_res[SV3_HR].th_res * 180.0f / 3.14f), (int)(sv_res[SV4_HL].th_res * 180.0f / 3.14f));
    //printf("cmd %6d ", (int)(vel_ctrl.vel_cmd * 180.0f / 3.14f));
    //printf("vel %6d %6d", (int)(sv_res[SV1_FR].vel_res), (int)(sv_res[SV2_FL].vel_res));
    
    //printf("omega_res %6d %6d ", (int)(sv_res[SV1_FR].vel_res), (int)(sv_res[SV2_FL].vel_res));
    //printf("cmd %6d ", (int)(vel_ctrl.vel_cmd * 10.0f));
    //printf("VREF %6d %6d ", (int16_t)(BVC.v_ref[SV1_FR]*1000),(int16_t)(BVC.v_ref[SV2_FL]*1000));
    printf("TREF %6d %6d ", (int16_t)(BVC.t_ref[SV1_FR]*1000),(int16_t)(BVC.t_ref[SV2_FL]*1000));
    //printf("velres %6d %6d ", (int)(sv_res[SV1_FR].vel_res * 100.0f), (int)(sv_res[SV2_FL].vel_res * 100.0f));
    //printf("vctx %6d %6d ", (int)(BVC.v_ref[0]),(int)(BVC.v_ref[1]));
    printf("IQ %6d %6d ", sv_data[SV1_FR].iq, sv_data[SV2_FL].iq);
    //printf("Isrc %6d %6d ", (int)(sv_res[SV1_FR].iq_res), (int)(sv_res[SV2_FL].iq_res));// 20240521現在、SVからはsrc電流がIqのところに来ている。
    printf("VOL %6d %6d ", (int)(sv_res[SV1_FR].vdc), (int)(sv_res[SV2_FL].vdc));
#endif

    /*
    printf("%x %x %x %x %x %x %x %x %x %x " , apmp_data.head_h
                                                , apmp_data.head_l
                                                , apmp_data.cmd
                                                , apmp_data.vx_cmd_h
                                                , apmp_data.vx_cmd_l                                                
                                                , apmp_data.vy_cmd_h
                                                , apmp_data.vy_cmd_l   
                                                , apmp_data.wz_cmd_h
                                                , apmp_data.wz_cmd_l
                                                , apmp_data.crc);
    //*/
    
    // printf("\n");
#if 0
    printf("mecp %6d(%5d) %6d(%5d) %6d(%5d) %6d(%5d) "
        ,(int)(MECP.wheel_radius_r * 1000000.0f) 
        ,apmp_data.wheel_radius_right
        ,(int)(MECP.wheel_radius_l * 1000000.0f) 
        ,apmp_data.wheel_radius_left
        ,(int)(MECP.tread          * 100000.0f) 
        ,apmp_data.tread
        ,(int)(imu_data.gyro_scale_gain*100000.0f)
        ,(int)((1.0f + ((float)((int32_t)(apmp_data.imu_gyro_scale_gain)) - 32768) * GYRO_SCALE_GAIN_INT16t_TO_FLOAT)*100000.0f));
#endif
#if 0
    printf("APMP %x %x %x %x "
        , apmp_data.wheel_radius_right
        , apmp_data.wheel_radius_left
        , apmp_data.tread
        , apmp_data.imu_gyro_scale_gain);
#endif
    /*
    printf("mpap %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x " , mpap_buff[0]
                                                , mpap_buff[1]
                                                , mpap_buff[2]
                                                , mpap_buff[3]
                                                , mpap_buff[4]                                               
                                                , mpap_buff[5]
                                                , mpap_buff[6]  
                                                , mpap_buff[7]
                                                , mpap_buff[8]
                                                , mpap_buff[9]
                                                , mpap_buff[10]
                                                , mpap_buff[11]
                                                , mpap_buff[12]
                                                , mpap_buff[13]
                                                , mpap_buff[14]
                                                , mpap_buff[15]
                                                , mpap_buff[16]
                                                , mpap_buff[17]
                                                , mpap_buff[18]
                                                , mpap_buff[19]
                                                , mpap_buff[20]
                                                , mpap_buff[21]
                                                , mpap_buff[22]
                                                , mpap_buff[28]
                                                );

    
    //*/

    ///* IMU
    //printf("imu %8x %d ",imu_data.uni_gyro[X_AXIS].i, imu_debug_flag);
    //imu_debug_flag = 0;
    //printf("IMU_PARSE %6x ",);

    //printf("IMU_PARSE %2x %2x %2x %2x ",mpap_buff[28], mpap_buff[29],mpap_buff[30], mpap_buff[31]);


    //*/

    ///*SV omega
    //printf("omega_cmd %6d %6d ", (int16_t)((mpap_buff[52]<<8) + mpap_buff[53]), (int16_t)((mpap_buff[56]<<8) + mpap_buff[57]) );
    //printf("omega_res %6d %6d ", (int16_t)((mpap_buff[54]<<8) + mpap_buff[55]), (int16_t)((mpap_buff[58]<<8) + mpap_buff[59]) );
    ///*

    //printf("gg %8d %6d " , (int32_t)(imu_data.gyro_scale_gain*1000000), ((uint16_t)(apmp_data.imu_gyro_scale_gain_h) << 8) + (apmp_data.imu_gyro_scale_gain_l));
    printf("flag %2d " , dbg_flag_mpap);

    //*/

    printf("\n");

}

void debug_command(int c){
    if(c == 'i'){
        //MC_state.state = MC_STATE_    IDLE;
        apmp_data.cmd = AP_MP_CMD_IDLE;
    }else if(c == 's'){
        if(MC_state.state == MC_STATE_IDLE){
            //MC_state.state = MC_STATE_START;
            apmp_data.cmd = AP_MP_CMD_RUN;
        }else{
            //MC_state.state = MC_STATE_STOP;
            apmp_data.cmd = AP_MP_CMD_IDLE;
        }
    }else if(c == 'u'){
        //vel_ctrl.vel_cmd += 1.0f;
    }else if(c == 'd'){
        //vel_ctrl.vel_cmd -= 1.0f;
    }else if(c == 'r'){
        //vel_ctrl.vel_cmd = - vel_ctrl.vel_cmd;
    }else if(c == 'X'){
        BVC.vel_cmd_dbg[X_AXIS] += 0.5f;
    }else if(c == 'x'){
        BVC.vel_cmd_dbg[X_AXIS] -= 0.5f;
    }else if(c == 'Y'){
        BVC.vel_cmd_dbg[Y_AXIS] += 0.1f;
    }else if(c == 'y'){
        BVC.vel_cmd_dbg[Y_AXIS] -= 0.1f;
    }else if(c == 'W'){
        BVC.vel_cmd_dbg[Z_AXIS] += 3.14f / 20.0f;
    }else if(c == 'w'){
        BVC.vel_cmd_dbg[Z_AXIS] -= 3.14f / 20.0f;
    }else if(c == '0'){
        BVC.vel_cmd_dbg[X_AXIS] = 0.0f;
        BVC.vel_cmd_dbg[Y_AXIS] = 0.0f;
        BVC.vel_cmd_dbg[Z_AXIS] = 0.0f;
    }
    
}
