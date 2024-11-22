#include <stm32h747xx.h>
#include "motion_controller.h"
#include "imu_common.h"
#include "imu_icm42688.h"
#include "comm_mpsv.h"
#include "comm_apmp.h"
#include "timer.h"
#include "gpio.h"

#include <stdbool.h>
#include "HW_type.h"
#include "vqfInstance.h"

MotionControlStatus_t MC_state = {
    .state = MC_STATE_IDLE,
    .time_count = 0,
};

BODY_VEL_CTRL_t BVC={
    .omega_wob = 2.0f * 3.14f * 2.5f,
    // .kp[0] = 2.0f * 3.14f * 15.0f,
    // .kp[1] = 2.0f * 3.14f * 15.0f,
    // .kp[2] = 2.0f * 3.14f * 15.0f,
    .kp[0] = 2.0f * 3.14f * 20.0f,
    .kp[1] = 2.0f * 3.14f * 0.0f,
    .kp[2] = 2.0f * 3.14f * 20.0f,
    .vel_cmd[0] = 0.0f,
    .vel_cmd[1] = 0.0f,
    .vel_cmd[2] = 0.0f,
    .vel_cmd_raw[0] = 0.0f,
    .vel_cmd_raw[1] = 0.0f,
    .vel_cmd_raw[2] = 0.0f,
    .vel_cmd_dbg[0] = 0.0f,
    .vel_cmd_dbg[1] = 0.0f,
    .vel_cmd_dbg[2] = 0.0f,
    .BVC_debug_flag = 0,
};

MECHA_PARAM_t MECP = {
    .wheel_radius_r = WHEEL_RADIUS,
    .wheel_radius_l = WHEEL_RADIUS,
    .tread = WHEEL_TREAD,
};

IMU_Data_t imu_data = {
    .gyro_scale_gain = 1.0f,
};

SV_Data_float_t sv_res[4];
ODOM_Data_t ODOM_data;

void BodyVelocityControl(void);
void BodyVelocityControl_2(void);
void BodyVelocityControl_3(void);
void MotionControl_IrqHandler();

void MotionControl_Init() {

  icm42688_RegisterReceiveDataCallback(&MotionControl_IrqHandler);
}

void MotionControl_IrqHandler(){

    //TEST_PF9(1);
    //IMUから受信したデータの読み取り処理
    imu_float_data l_imu = {0};
    l_imu = icm42688_GetDataFloat();

    imu_data.acc[X_AXIS] = ACC_X_DIRECTION * l_imu.acceleration_mg_x;
    imu_data.acc[Y_AXIS] = ACC_Y_DIRECTION * l_imu.acceleration_mg_y;
    imu_data.acc[Z_AXIS] = ACC_Z_DIRECTION * l_imu.acceleration_mg_z;
    imu_data.gyro[X_AXIS] = GYRO_X_DIRECTION * l_imu.angular_rate_mrads_x;
    imu_data.gyro[Y_AXIS] = GYRO_Y_DIRECTION * l_imu.angular_rate_mrads_y;
    imu_data.gyro[Z_AXIS] = GYRO_Z_DIRECTION * l_imu.angular_rate_mrads_z;

    //速度指令値代入
    BVC.vel_cmd_raw[X_AXIS] = COMM_DIR_X * ((float)((int32_t)(apmp_data.vx_cmd_l + (uint16_t)(apmp_data.vx_cmd_h << 8)) - 32768) * VXY_INT16t_TO_MPS + BVC.vel_cmd_dbg[X_AXIS]);  // [m/s]
    BVC.vel_cmd_raw[Z_AXIS] = COMM_DIR_Z * ((float)((int32_t)(apmp_data.wz_cmd_l + (uint16_t)(apmp_data.wz_cmd_h << 8)) - 32768) * WZ_INT16t_TO_RADPS  + BVC.vel_cmd_dbg[Z_AXIS]); // [rad/s]

    if (fabsf(BVC.vel_cmd_raw[X_AXIS]) > 0.001f || fabsf(BVC.vel_cmd_raw[Z_AXIS]) > (0.0001f * SMC_PI_F / 180.0f)){
        vqf_SetRestBiasEstEnabled(false);
    } else {
        // 目標速度のクリア
        BVC.vel_cmd_raw[X_AXIS] = 0.0f;
        BVC.vel_cmd_raw[Z_AXIS] = 0.0f;
        vqf_SetRestBiasEstEnabled(true);
    }

    // VQFで姿勢推定
    float imu_gyro[3] = {imu_data.gyro[X_AXIS], -imu_data.gyro[Y_AXIS], imu_data.gyro[Z_AXIS] * imu_data.gyro_scale_gain};
    float imu_acc[3]  = {imu_data.acc[X_AXIS],  -imu_data.acc[Y_AXIS],  -imu_data.acc[Z_AXIS]};
    vqf_Update(imu_gyro, imu_acc);

    // バイアス推定値を取得
    float bias[3];
    vqf_GetBiasEstimate(bias);

    float vqf_gyro[3] = {imu_gyro[0] - bias[0], -(imu_gyro[1] - bias[1]), (imu_gyro[2] - bias[2])};

    // 姿勢をクォータニオンで取得
    float quat[4];
    vqf_GetQuat6D(quat);
    
    for (int i=0; i<3; i++){
        if (imu_data.calib_end){
            imu_data.acc[i]  -= imu_data.acc_bias[i];
            imu_data.gyro[i] -= imu_data.gyro_bias[i];

            //imu_data.theta[i] += imu_data.gyro[i] / MOTION_CONTROL_FREQ;
            //imu_data.theta[i] = MC_CLIP_PI(imu_data.theta[i]);
        } else {
            //imu_data.theta[i] = 0.0f;
        }
    }

    // キャリブが終わっていたらゲイン調整(APから遅れて貰うため)
    if (imu_data.calib_end) {
        imu_data.gyro[Z_AXIS] *= imu_data.gyro_scale_gain;
    }

    imu_data.uni_acc[X_AXIS].f  = imu_data.acc[X_AXIS];
    imu_data.uni_acc[Y_AXIS].f  = imu_data.acc[Y_AXIS];
    imu_data.uni_acc[Z_AXIS].f  = imu_data.acc[Z_AXIS];
    imu_data.uni_gyro[X_AXIS].f = imu_data.gyro[X_AXIS];
    imu_data.uni_gyro[Y_AXIS].f = imu_data.gyro[Y_AXIS];
    imu_data.uni_gyro[Z_AXIS].f = imu_data.gyro[Z_AXIS];

    //svから受信したデータの読み取り処理
    for(int i = 0; i < COMM_SV_NUM; i++){
        sv_res[i].iq_res = (float)sv_data[i].iq * 25.0f / 32767.0f*100; //[0.01A]
        sv_res[i].th_res = (float)sv_data[i].mr * 3.14159265358979f / 32767.0f;
        //sv_res[i].vel_res = (float)sv_data[i].hall * 2.0f * 25.0f * 3.14159265358979f / 65535.0f;
        sv_res[i].vel_res = (float)sv_data[i].hall;
        sv_res[i].vdc = (float)(sv_data[i].vdc);
    }

#ifdef MC_UNCONTROLLABLE_EMMERGENCY
    static uint16_t uncontroll_count = 0;
    if( (fabsf(sv_res[SV1_FR].vel_res) > MC_OVER_MOTOR_OMEGA) && (fabsf(sv_res[SV2_FL].vel_res) > MC_OVER_MOTOR_OMEGA) ){
        uncontroll_count += 1;
    }else{
        if(uncontroll_count > 1){
            uncontroll_count -= 1;
        }
    }

    if(uncontroll_count > MC_UNCONTROLLABLE_LIMIT){
        uncontroll_count = 0;
        apmp_data.cmd = AP_MP_CMD_IDLE;
    }

#endif

#if (COMM_APMP_SW == COMM_APMP_ON)
    // //apから受信したデータの読み取り処理
#if 0
    static uint8_t flag = 0;
    if(apmp_data.cmd == AP_MP_CMD_IDLE){
        MC_state.state = MC_STATE_STOP;
        flag = 0;
    }else if(apmp_data.cmd == AP_MP_CMD_RUN){
        if(flag == 0){
            MC_state.state = MC_STATE_START;
        }
        flag = 1;
    }
#endif
    //指示値リミット
    BVC.vel_cmd_raw[X_AXIS] = LIM_MIN_MAX(BVC.vel_cmd_raw[X_AXIS], - BODY_VX_LIMIT, BODY_VX_LIMIT);
    BVC.vel_cmd_raw[Y_AXIS] = LIM_MIN_MAX(BVC.vel_cmd_raw[Y_AXIS], - BODY_VY_LIMIT, BODY_VY_LIMIT);
    BVC.vel_cmd_raw[Z_AXIS] = LIM_MIN_MAX(BVC.vel_cmd_raw[Z_AXIS], - BODY_WZ_LIMIT, BODY_WZ_LIMIT);

    // ************************************
    // 中心方向加速度リミット
    // ************************************
    const float a_max = 6.0f;   // 6.0 m/s
    float x_vel_ref = BVC.vel_ref[X_AXIS];
    //float z_vel_ref = BVC.vel_ref[Z_AXIS];
    float z_vel_ref = BVC.vel_cmd[Z_AXIS];

    if (fabsf(x_vel_ref * z_vel_ref) > a_max) {
        x_vel_ref = (x_vel_ref < 0.0f ? -1.0f : 1.0f) * a_max / fabsf(z_vel_ref);
        BVC.vel_ref[X_AXIS] = x_vel_ref;
        BVC.BVC_debug_flag |= FLAG_BVC_LIM_CENTORIPETAL_FORCE;
    }

#if 1
    // 加速度リミット
    //const float max_acc = 1.5f;     // m/s/s
    //const float max_acc = 6.0f;     // m/s/s
    const float max_acc = 2.5f;     // m/s/s
    BVC.vel_cmd[X_AXIS] = MC_DELTA_LIMIT(BVC.vel_cmd[X_AXIS], BVC.vel_cmd_raw[X_AXIS], max_acc / MOTION_CONTROL_FREQ);
    if(BVC.vel_cmd[X_AXIS] != BVC.vel_cmd_raw[X_AXIS]){
        BVC.BVC_debug_flag |= FLAG_BVC_LIM_ACC;
    }

    // 角加速度リミット
    //const float max_arate = 20.0f;   // rad/s/s
    //const float max_arate = 100.0f;   // rad/s/s
    const float max_arate = 15.0f;   // rad/s/s
    BVC.vel_cmd[Z_AXIS] = MC_DELTA_LIMIT(BVC.vel_cmd[Z_AXIS], BVC.vel_cmd_raw[Z_AXIS], max_arate / MOTION_CONTROL_FREQ);


#else
    BVC.vel_cmd[X_AXIS] = BVC.vel_cmd_raw[X_AXIS]; 
    BVC.vel_cmd[Z_AXIS] = BVC.vel_cmd_raw[Z_AXIS];
#endif

#elif (COMM_APMP_SW == COMM_APMP_OFF)
#endif

    // ************************************
    // 速度指示値から位置指令値(VPOS)を生成
    // ************************************
    // 自己移動量を計算する
    float delta_yaw = BVC.vel_cmd[Z_AXIS] / MOTION_CONTROL_FREQ;
    float pos_cmd_body_x = BVC.vel_cmd[X_AXIS] / MOTION_CONTROL_FREQ * cosf(BVC.pos_cmd[Z_AXIS] + delta_yaw / 2.0f);
    float pos_cmd_body_y = BVC.vel_cmd[X_AXIS] / MOTION_CONTROL_FREQ * sinf(BVC.pos_cmd[Z_AXIS] + delta_yaw / 2.0f);

    // ローカル座標系の自己位置を更新する
    BVC.pos_cmd[X_AXIS] += pos_cmd_body_x;
    BVC.pos_cmd[Y_AXIS] += pos_cmd_body_y;
    //BVC.pos_cmd[Z_AXIS] += delta_yaw;
    BVC.pos_cmd[Z_AXIS] = MC_CLIP_PI(BVC.pos_cmd[Z_AXIS] + delta_yaw);

    // ************************************
    // オドメトリの計算
    // ************************************

    // ボディーのX軸に合わせる
    ODOM_data.sv1_delta_len = (int16_t)(sv_data[SV1_FR].mr - ODOM_data.sv1_mr_prev) * (SMC_PI_F * MECP.wheel_radius_r / 32767.0f) * SV1_DIRECTION;
    ODOM_data.sv2_delta_len = (int16_t)(sv_data[SV2_FL].mr - ODOM_data.sv2_mr_prev) * (SMC_PI_F * MECP.wheel_radius_l / 32767.0f) * SV2_DIRECTION;
    ODOM_data.sv1_mr_prev = sv_data[SV1_FR].mr;
    ODOM_data.sv2_mr_prev = sv_data[SV2_FL].mr;

    // ホイールの移動量から回転角度を求める
    // V1dt=(R+d)*ωdt、V2dt=(R-d)*ωdt
    // R=V1dt/ωdt-d、R=V2dt/ωdt+d
    // ωdt=(V1dt-V2dt)/2d
#if 0
    ODOM_data.delta_theta = (ODOM_data.sv1_delta_len - ODOM_data.sv2_delta_len) / (MECP.tread);
#elif 0
    ODOM_data.delta_theta = imu_data.gyro[Z_AXIS] / MOTION_CONTROL_FREQ;
#else
    ODOM_data.delta_theta_odom = (ODOM_data.sv1_delta_len - ODOM_data.sv2_delta_len) / (MECP.tread);
    //ODOM_data.delta_theta_gyro = imu_data.gyro[Z_AXIS] / MOTION_CONTROL_FREQ;
    ODOM_data.delta_theta_gyro = vqf_gyro[2] / MOTION_CONTROL_FREQ;
    ODOM_data.delta_theta = ODOM_data.delta_theta_gyro;
#endif

    // ホイールの平均の移動量を求める
    // Vdt=R*ωdt、V1dt=(R+d)*ωdt、V2dt=(R-d)*ωdt
    // V1dt=Vdt+d*ωdt、V2dt=Vdt-d*ωdt
    // d*ωdt=(V1dt-Vdt)=(Vdt-V2dt)
    // Vdt=(V1dt+V2dt)/2
    ODOM_data.delta_len = (ODOM_data.sv1_delta_len + ODOM_data.sv2_delta_len) / 2.0f;

    // 自己移動量を計算する
    double pos_body_x = ODOM_data.delta_len * cos(BVC.pos_res[Z_AXIS] + ODOM_data.delta_theta / 2.0f);
    double pos_body_y = ODOM_data.delta_len * sin(BVC.pos_res[Z_AXIS] + ODOM_data.delta_theta / 2.0f);

    // ローカル座標系の自己位置を更新する
    BVC.pos_res[X_AXIS] += pos_body_x;
    BVC.pos_res[Y_AXIS] += pos_body_y;
    //BVC.pos_res[Z_AXIS] += ODOM_data.delta_theta;
#if 1
    //BVC.pos_res[Z_AXIS] = MC_CLIP_PI(BVC.pos_res[Z_AXIS] + ODOM_data.delta_theta);
    BVC.pos_res[Z_AXIS] = vqf_QuatToEulerZ(quat);
#else
    BVC.pos_res[Z_AXIS] = imu_data.theta[Z_AXIS];
#endif

    // 速度はボディ座標系で返す
    //BVC.vel_res[X_AXIS] = ODOM_data.delta_len * MOTION_CONTROL_FREQ;
    //BVC.vel_res[Y_AXIS] = 0.0f;
    //BVC.vel_res[Z_AXIS] = ODOM_data.delta_theta * MOTION_CONTROL_FREQ;
    //BVC.vel_res[Z_AXIS] = imu_data.gyro[Z_AXIS];

    // 速度にLPFを掛ける
    const float x_vel_LPF_fc = 50.0f;
    const float z_vel_LPF_fc = 150.0f;
    //BVC.x_vel_res_LPF += (ODOM_data.delta_len * MOTION_CONTROL_FREQ - BVC.x_vel_res_LPF) * x_vel_LPF_fc / MOTION_CONTROL_FREQ;
    BVC.vel_res[X_AXIS] += (ODOM_data.delta_len - BVC.vel_res[X_AXIS] / MOTION_CONTROL_FREQ) * x_vel_LPF_fc;
    BVC.vel_res[Y_AXIS] = 0.0f;
#if 1
    BVC.vel_res[Z_AXIS] += (ODOM_data.delta_theta - BVC.vel_res[Z_AXIS] / MOTION_CONTROL_FREQ) * z_vel_LPF_fc; 
#else
    BVC.vel_res[Z_AXIS] += (imu_data.gyro[Z_AXIS] - BVC.vel_res[Z_AXIS]) * z_vel_LPF_fc / MOTION_CONTROL_FREQ; 
#endif

    // ************************************
    // 前後速度への指示値を修正
    // ************************************
    float x_error = BVC.pos_cmd[X_AXIS] - BVC.pos_res[X_AXIS];
    float y_error = BVC.pos_cmd[Y_AXIS] - BVC.pos_res[Y_AXIS];
    
    ODOM_data.x_error_body = x_error * cosf(-BVC.pos_res[Z_AXIS]) - y_error * sinf(-BVC.pos_res[Z_AXIS]);
    ODOM_data.y_error_body = x_error * sinf(-BVC.pos_res[Z_AXIS]) + y_error * cosf(-BVC.pos_res[Z_AXIS]);

#if 0
    const float vel_kp  = 8.0f;
    const float vel_kff = 0.0f;
#else
    const float vel_kp  = 4.0f;
    const float vel_kff = 1.0f;
#endif
    float vel_ref_ff_x = BVC.vel_cmd[X_AXIS] * vel_kff;

    BVC.vel_ref[X_AXIS] = ODOM_data.x_error_body * vel_kp + vel_ref_ff_x;

    //指示値リミット
    BVC.vel_ref[X_AXIS] = LIM_MIN_MAX(BVC.vel_ref[X_AXIS], - BODY_VX_LIMIT, BODY_VX_LIMIT);

    // ************************************
    // Yawの指示値を修正
    // ************************************
    //const float yaw_kp  = 14.0f;
    //const float yaw_kp  = 13.0f;
#if 0
    const float yaw_dp  = 0.0f;
    const float yaw_kp  = 14.0f;
    const float yaw_kff = 0.3f;
#else
    const float yaw_dp  = 10.0f;
    const float yaw_kp  = 10.0f;
    const float yaw_kff = 1.0f;
#endif

    float omega_adj = ODOM_data.y_error_body * yaw_dp;
    BVC.omega_ref_adj_y = (fabsf(BVC.vel_cmd[X_AXIS]) < 1.0 ? omega_adj * fabsf(BVC.vel_cmd[X_AXIS]) : omega_adj);

    float vel_ref_ff_z = BVC.vel_cmd[Z_AXIS] * yaw_kff;

#if 1
    //BVC.vel_ref[Z_AXIS] = (BVC.pos_cmd[Z_AXIS] - BVC.pos_res[Z_AXIS]) * yaw_kp + BVC.vel_cmd[Z_AXIS] * yaw_kff;
    BVC.vel_ref[Z_AXIS] = (MC_CLIP_PI(BVC.pos_cmd[Z_AXIS] - BVC.pos_res[Z_AXIS])) * yaw_kp + vel_ref_ff_z + BVC.omega_ref_adj_y;
#else
    if (vqf_GetRestDetected()){
        BVC.vel_ref[Z_AXIS] = 0.0f;
    } else {
        BVC.vel_ref[Z_AXIS] = (MC_CLIP_PI(BVC.pos_cmd[Z_AXIS] - BVC.pos_res[Z_AXIS])) * yaw_kp + vel_ref_ff_z + BVC.omega_ref_adj_y;
    }        
#endif

    //指示値リミット
    BVC.vel_ref[Z_AXIS] = LIM_MIN_MAX(BVC.vel_ref[Z_AXIS], - BODY_WZ_LIMIT, BODY_WZ_LIMIT);

#if 0
    // ************************************
    // Yawの空転検出
    // ************************************
    // Gyroとオドメトリの角速度差が一定以上
    if(fabsf(ODOM_data.delta_theta_odom) > (180.0f * SMC_PI_F / 180.0f / MOTION_CONTROL_FREQ)) {
        float over_dtheta = fabsf(ODOM_data.delta_theta_odom) - (180.0f * SMC_PI_F / 180.0f / MOTION_CONTROL_FREQ);
        ODOM_data.dtheta_ratio = (over_dtheta > (20.0f * SMC_PI_F / 180.0f / MOTION_CONTROL_FREQ) ? 0.0f :
                                  1.0f - (over_dtheta / 20.0f * SMC_PI_F / 180.0f / MOTION_CONTROL_FREQ));
    } else {
        ODOM_data.dtheta_ratio = 1.0f;
    }

    BVC.vel_ref[Z_AXIS] *= ODOM_data.dtheta_ratio;   
#endif

    // ************************************
    // VPOSの修正
    // ************************************
    if (vel_kp > 0.000001f){
#if 0
        BVC.pos_cmd[X_AXIS] = (BVC.vel_ref[X_AXIS] - vel_ref_ff_x) / vel_kp + BVC.pos_res[X_AXIS];
#else
        float x_error_body = (BVC.vel_ref[X_AXIS] - vel_ref_ff_x) / vel_kp;
        float x_error = x_error_body * cosf(BVC.pos_res[Z_AXIS]) - ODOM_data.y_error_body * sinf(BVC.pos_res[Z_AXIS]);
        float y_error = x_error_body * sinf(BVC.pos_res[Z_AXIS]) + ODOM_data.y_error_body * cosf(BVC.pos_res[Z_AXIS]);
        BVC.pos_cmd[X_AXIS] = x_error + BVC.pos_res[X_AXIS];
        BVC.pos_cmd[Y_AXIS] = y_error + BVC.pos_res[Y_AXIS];
#endif
    }

    if (vqf_GetRestDetected()){
        // 静止判定時は角速度の指示0でVPOSの角度も修正しない
        BVC.vel_ref[Z_AXIS] = 0.0f;
    }else if (yaw_kp > 0.000001f){
        //BVC.pos_cmd[Z_AXIS] = MC_CLIP_PI((BVC.vel_ref[Z_AXIS] - vel_ref_ff_z) / yaw_kp + BVC.pos_res[Z_AXIS]);
        BVC.pos_cmd[Z_AXIS] = MC_CLIP_PI((BVC.vel_ref[Z_AXIS] - vel_ref_ff_z - BVC.omega_ref_adj_y) / yaw_kp + BVC.pos_res[Z_AXIS]);
    }

    // ************************************

    //制御処理
    //BodyVelocityControl_2();
    BodyVelocityControl_3();

    //SVへの指示
    COM_MP_TO_SV * cmd_mp_sv = get_send_cmd_handle();
    COM_MP_TO_SV * mode_mp_sv = get_send_mode_handle();

    int16_t cmd[COMM_SV_NUM] = {0, 0};
    uint8_t to_sv_mode[COMM_SV_NUM] = {TO_IDLE_MODE, TO_IDLE_MODE};
    const float acc_lpf_fc = 1.0f;
    const float gyro_lpf_fc = 1.0f;
    const float acc_max_calib_noise = 2.0f;     // m/s/s
    const float gyro_max_calib_noise = 0.02f;    // rad/s
    //const float gyro_max_calib_noise = 1.0f;    // rad/s

    // 停止指示なら問答無用にストップへステート遷移
    if(apmp_data.cmd == AP_MP_CMD_IDLE && (MC_state.state == MC_STATE_START || MC_state.state == MC_STATE_RUN)){
        MC_state.state = MC_STATE_STOP;
    }
    // ステート制御
    switch(MC_state.state){
        case MC_STATE_INIT: // 初期化
            imu_data.calib_count = 0;
            imu_data.calib_end = false;
            for (int i=0; i<3; i++) {
                imu_data.acc_bias[i] = imu_data.acc[i];
                imu_data.gyro_bias[i] = imu_data.gyro[i];
                imu_data.acc_min[i] = imu_data.acc[i];
                imu_data.gyro_min[i] = imu_data.gyro[i];
                imu_data.acc_max[i] = imu_data.acc[i];
                imu_data.gyro_max[i] = imu_data.gyro[i];
                //imu_data.theta[i] = 0.0f;
#if 0
                imu_data.gyro_scale_gain = 1.0f + (float)((int32_t)((uint16_t)(apmp_data.imu_gyro_scale_gain_h) << 8) + (apmp_data.imu_gyro_scale_gain_l) - 32768)*GYRO_SCALE_GAIN_INT16t_TO_FLOAT; //機体ごとのスケールゲインをAPからもらう
            }
            if(imu_data.gyro_scale_gain>1.1 || imu_data.gyro_scale_gain<0.9){
                imu_data.gyro_scale_gain = IMU_GYRO_SCALE_GAIN; //うまく受信できていなかったらデフォルト値で初期化
#endif
            }
            to_sv_mode[SV1_FR] = TO_IDLE_MODE;
            to_sv_mode[SV2_FL] = TO_IDLE_MODE;
            send_mode_broadcast_data(mode_mp_sv, COMM_SV_NUM, to_sv_mode, MOTOR_TEMP);  //Initで明示的にSVを停止させる

            MC_state.state = MC_STATE_CALIB;
            break;
        case MC_STATE_CALIB:
            for (int i=0; i<3; i++) {
                imu_data.acc_bias[i] += (imu_data.acc[i] - imu_data.acc_bias[i]) * acc_lpf_fc / MOTION_CONTROL_FREQ;
                imu_data.gyro_bias[i] += (imu_data.gyro[i] - imu_data.gyro_bias[i]) * gyro_lpf_fc / MOTION_CONTROL_FREQ;
                //imu_data.theta[i] = 0.0f;

                if (imu_data.acc_min[i] > imu_data.acc[i]) {
                    imu_data.acc_min[i] = imu_data.acc[i];
                }
                if (imu_data.gyro_min[i] > imu_data.gyro[i]) {
                    imu_data.gyro_min[i] = imu_data.gyro[i];
                }
                if (imu_data.acc_max[i] < imu_data.acc[i]) {
                    imu_data.acc_max[i] = imu_data.acc[i];
                }
                if (imu_data.gyro_max[i] < imu_data.gyro[i]) {
                    imu_data.gyro_max[i] = imu_data.gyro[i];
                }

                if ((imu_data.acc_max[i] - imu_data.acc_min[i] > acc_max_calib_noise) ||
                    (imu_data.gyro_max[i] - imu_data.gyro_min[i] > gyro_max_calib_noise) ){
                    BVC.BVC_debug_flag |= FLAG_BVC_IMU_CALIB_ERR;
                    MC_state.state = MC_STATE_INIT;
                    break;
                }
            }

            imu_data.calib_count++;

            if (imu_data.calib_count > MOTION_CONTROL_FREQ * 5){
                MC_state.state = MC_STATE_IDLE;
                imu_data.calib_end = true;
                to_sv_mode[SV1_FR] = TO_FORCE_MODE;
                to_sv_mode[SV2_FL] = TO_FORCE_MODE;
                send_mode_broadcast_data(mode_mp_sv, COMM_SV_NUM, to_sv_mode, MOTOR_TEMP);         
            }
            break;
        case MC_STATE_IDLE:
            cmd[SV1_FR] = 0;
            cmd[SV2_FL] = 0;
            vqf_ResetQuat();
            send_cmd_broadcast_data(cmd_mp_sv, FORCE_CNTRL_ALL, COMM_SV_NUM, cmd, MOTOR_TEMP);
            if (imu_data.calib_end == false){
                MC_state.state = MC_STATE_INIT;
            } else if (apmp_data.cmd == AP_MP_CMD_RUN) {
                MC_state.state = MC_STATE_START;
            }
            break;
        case MC_STATE_START://SVを駆動モードに入れてStateをrunに変更する
#if 0 
            to_sv_mode[SV1_FR] = TO_VEL_MODE;
            to_sv_mode[SV2_FL] = TO_VEL_MODE;
            // to_sv_mode[SV3_HR] = TO_VEL_MODE;
            // to_sv_mode[SV4_HL] = TO_VEL_MODE;
#elif 0     //サーボオフにしてオドメトリ確認したいときは0にする 
            to_sv_mode[SV1_FR] = TO_FORCE_MODE;
            to_sv_mode[SV2_FL] = TO_FORCE_MODE;

#else

#endif

            // APから受け取ったメカパラメータの取り込み
            MECP.wheel_radius_r = ((float)apmp_data.wheel_radius_right) / 1000000.0f;
            MECP.wheel_radius_l = ((float)apmp_data.wheel_radius_left ) / 1000000.0f;
            MECP.tread =          ((float)apmp_data.tread )             / 100000.0f;
            imu_data.gyro_scale_gain = 1.0f + ((float)((int32_t)(apmp_data.imu_gyro_scale_gain)) - 32768) * GYRO_SCALE_GAIN_INT16t_TO_FLOAT; //機体ごとのスケールゲインをAPからもらう
            //send_mode_broadcast_data(mode_mp_sv, COMM_SV_NUM, to_sv_mode, MOTOR_TEMP);
            MC_state.state = MC_STATE_RUN;
            break;
        case MC_STATE_RUN:
#if 0
            //cmd[SV1_FR] = (int16_t)(35);  //for debug
            //cmd[SV2_FL] = (int16_t)(35);  //for debug
            cmd[SV1_FR] = (int16_t)(BVC.v_ref[SV1_FR] * 100); //[0.01Hz]
            cmd[SV2_FL] = (int16_t)(BVC.v_ref[SV2_FL] * 100); //[0.01Hz]
            // cmd[SV3_HR] = (int16_t)(BVC.i_ref[SV3_HR] * 32767.0f / 25.0f);
            // cmd[SV4_HL] = (int16_t)(BVC.i_ref[SV4_HL] * 32767.0f / 25.0f);
            send_cmd_broadcast_data(cmd_mp_sv, VEL_CNTRL_ALL, COMM_SV_NUM, cmd, MOTOR_TEMP);
#else
 #if 0
            // 本当は速度ではなくトルク的なものを送る
            cmd[SV1_FR] = (int16_t)(BVC.v_ref[SV1_FR] * 1000); //[0.01Hz]
            cmd[SV2_FL] = (int16_t)(BVC.v_ref[SV2_FL] * 1000); //[0.01Hz]
 #else
            // 本当は速度ではなくトルク的なものを送る
            cmd[SV1_FR] = (int16_t)(BVC.t_ref[SV1_FR] * 10000.0f); //[1mNm]
            cmd[SV2_FL] = (int16_t)(BVC.t_ref[SV2_FL] * 10000.0f); //[1mNm]
#endif
            send_cmd_broadcast_data(cmd_mp_sv, FORCE_CNTRL_ALL, COMM_SV_NUM, cmd, MOTOR_TEMP);
#endif
            break;
        case MC_STATE_STOP:
            to_sv_mode[SV1_FR] = TO_IDLE_MODE;
            to_sv_mode[SV2_FL] = TO_IDLE_MODE;
            // to_sv_mode[SV3_HR] = TO_IDLE_MODE;
            // to_sv_mode[SV4_HL] = TO_IDLE_MODE;
            send_mode_broadcast_data(mode_mp_sv, COMM_SV_NUM, to_sv_mode, MOTOR_TEMP);
            //MC_state.state = MC_STATE_IDLE;
            MC_state.state = MC_STATE_INIT;
            break;
    }

    // if(comm_sv_err_cnt > 0){
    //     comm_sv_err_cnt--;
    // }
    

    MC_state.time_count++;
    if(MC_state.time_count % 16 == 0){
        send_data_on_mp_to_ap();//send start
    }

 //   TEST_PF9(0);
}


void BodyVelocityControl_3(void){
    static int last_state = MC_STATE_IDLE;

    if(MC_state.state != MC_STATE_RUN){
        // 速度指示値
        BVC.vel_cmd_raw[X_AXIS] = 0.0f;
        BVC.vel_cmd_raw[Y_AXIS] = 0.0f;
        BVC.vel_cmd_raw[Z_AXIS] = 0.0f;
        BVC.vel_cmd[X_AXIS] = 0.0f;
        BVC.vel_cmd[Y_AXIS] = 0.0f;
        BVC.vel_cmd[Z_AXIS] = 0.0f;
        BVC.vel_cmd_dbg[X_AXIS] = 0.0f;
        BVC.vel_cmd_dbg[Y_AXIS] = 0.0f;
        BVC.vel_cmd_dbg[Z_AXIS] = 0.0f;
        BVC.vel_ref[X_AXIS] = 0.0f;
        BVC.vel_ref[Y_AXIS] = 0.0f;
        BVC.vel_ref[Z_AXIS] = 0.0f;
        BVC.vel_res[X_AXIS] = 0.0f;
        BVC.vel_res[Y_AXIS] = 0.0f;
        BVC.vel_res[Z_AXIS] = 0.0f;
        BVC.vel_res_z1[X_AXIS] = 0.0f;
        BVC.vel_res_z1[Y_AXIS] = 0.0f;
        BVC.vel_res_z1[Z_AXIS] = 0.0f;
        BVC.pos_cmd[X_AXIS] = 0.0f;
        BVC.pos_cmd[Y_AXIS] = 0.0f;
        BVC.pos_cmd[Z_AXIS] = 0.0f;
        BVC.pos_res[X_AXIS] = 0.0f;
        BVC.pos_res[Y_AXIS] = 0.0f;
        BVC.pos_res[Z_AXIS] = 0.0f;
        BVC.acc_lpf[Z_AXIS] = 0.0f;
        BVC.acc_dis_lpf[X_AXIS] = 0.0f;
        BVC.acc_dis_lpf[Y_AXIS] = 0.0f;
        BVC.acc_dis_lpf[Z_AXIS] = 0.0f;
        //return;
    }
    else {
        static float x_vel_i_sum = 0.0f;
        static float z_vel_i_sum = 0.0f;
        
        if(last_state != MC_STATE_RUN){
            x_vel_i_sum = 0;
            z_vel_i_sum = 0;
        }

        const float pid_vel_x_kp = 150.0f;
        const float pid_vel_x_ki = 0.0f / MOTION_CONTROL_FREQ;

        const float pid_vel_z_kp = 250.0f;
        const float pid_vel_z_ki = 0.0f / MOTION_CONTROL_FREQ;

#if 1
        // 速度制御
        float x_vel_err = BVC.vel_ref[X_AXIS] - BVC.vel_res[X_AXIS];
        float z_vel_err = BVC.vel_ref[Z_AXIS] - BVC.vel_res[Z_AXIS];

        // 積分
        float x_vel_i = x_vel_err * pid_vel_x_ki;
        float z_vel_i = z_vel_err * pid_vel_z_ki;

        // 加速度を計算
        float x_acc = x_vel_err * pid_vel_x_kp + x_vel_i + x_vel_i_sum;
        float z_acc = z_vel_err * pid_vel_z_kp + z_vel_i + z_vel_i_sum;

        float x_force = x_acc * BODY_WEIGHT;                        // F = ma
        float z_force = z_acc * BODY_Z_INERTIA / (MECP.tread / 2.0f);  // T = ω'I、F = T / L

        // それぞれのホイールに掛かる力を算出
        float SV1_force = (x_force + z_force) / 2.0f;
        float SV2_force = (x_force - z_force) / 2.0f;

        const float max_force = 20.0f;     // N
        //const float max_force = 30.0f;     // N
        //const float max_force = 15.0f;

        // 力リミット時に積分が増加方向なら積分しない
        x_vel_i_sum = ((fabsf(SV1_force) > max_force || fabsf(SV2_force) > max_force) && (x_acc * x_vel_i > 0.0f) ? x_vel_i_sum : x_vel_i + x_vel_i_sum);
        z_vel_i_sum = ((fabsf(SV1_force) > max_force || fabsf(SV2_force) > max_force) && (z_acc * z_vel_i > 0.0f) ? z_vel_i_sum : z_vel_i + z_vel_i_sum);
#else
        const float max_force = 20.0f;     // N
        //const float max_force = 30.0f;     // N
        //const float max_force = 15.0f;

        float SV1_force;
        float SV2_force;

        if (vqf_GetRestDetected()){
            SV1_force = 0.0f;
            SV2_force = 0.0f;
            x_vel_i_sum = 0.0f;
            z_vel_i_sum = 0.0f;
        } else {
            // 速度制御
            float x_vel_err = BVC.vel_ref[X_AXIS] - BVC.vel_res[X_AXIS];
            float z_vel_err = BVC.vel_ref[Z_AXIS] - BVC.vel_res[Z_AXIS];

            // 積分
            float x_vel_i = x_vel_err * pid_vel_x_ki;
            float z_vel_i = z_vel_err * pid_vel_z_ki;

            // 加速度を計算
            float x_acc = x_vel_err * pid_vel_x_kp + x_vel_i + x_vel_i_sum;
            float z_acc = z_vel_err * pid_vel_z_kp + z_vel_i + z_vel_i_sum;

            float x_force = x_acc * BODY_WEIGHT;                        // F = ma
            float z_force = z_acc * BODY_Z_INERTIA / (MECP.tread / 2.0f);  // T = ω'I、F = T / L

            // それぞれのホイールに掛かる力を算出
            SV1_force = (x_force + z_force) / 2.0f;
            SV2_force = (x_force - z_force) / 2.0f;

            // 力リミット時に積分が増加方向なら積分しない
            x_vel_i_sum = ((fabsf(SV1_force) > max_force || fabsf(SV2_force) > max_force) && (x_acc * x_vel_i > 0.0f) ? x_vel_i_sum : x_vel_i + x_vel_i_sum);
            z_vel_i_sum = ((fabsf(SV1_force) > max_force || fabsf(SV2_force) > max_force) && (z_acc * z_vel_i > 0.0f) ? z_vel_i_sum : z_vel_i + z_vel_i_sum);
        }
#endif

#if 0
        // ホイールに掛かる力リミット
        SV1_force = LIM_MIN_MAX(SV1_force, - max_force, max_force);
        SV2_force = LIM_MIN_MAX(SV2_force, - max_force, max_force);
#else
        // ホイールに掛かる力を左右別々にリミット
        float over_force1 = 0.0f;
        float over_force2 = 0.0f;

        if (SV1_force - max_force > over_force1) {
            over_force1 = SV1_force - max_force;
            BVC.BVC_debug_flag |= FLAG_BVC_LIM_MOT_TORQUE_R;
        } else if (SV1_force + max_force < over_force1){
            over_force1 = SV1_force + max_force;
            BVC.BVC_debug_flag |= FLAG_BVC_LIM_MOT_TORQUE_R;
        }

        if (SV2_force - max_force > over_force2) {
            over_force2 = SV2_force - max_force;
            BVC.BVC_debug_flag |= FLAG_BVC_LIM_MOT_TORQUE_L;
        } else if (SV2_force + max_force < over_force2){
            over_force2 = SV2_force + max_force;
            BVC.BVC_debug_flag |= FLAG_BVC_LIM_MOT_TORQUE_L;
        }

        SV1_force -= over_force1;
        SV2_force -= over_force2;
#endif

        // 力変化量リミット
        const float max_delta_force = 1000.0f;     // N/s
        ODOM_data.SV1_force = MC_DELTA_LIMIT(ODOM_data.SV1_force, SV1_force, max_delta_force / MOTION_CONTROL_FREQ);
        ODOM_data.SV2_force = MC_DELTA_LIMIT(ODOM_data.SV2_force, SV2_force, max_delta_force / MOTION_CONTROL_FREQ);

        // 力をトルクに変換
        BVC.t_ref[SV1_FR] = ODOM_data.SV1_force * MECP.wheel_radius_r * SV1_DIRECTION;
        BVC.t_ref[SV2_FL] = ODOM_data.SV2_force * MECP.wheel_radius_l * SV2_DIRECTION;
    }
    last_state = MC_state.state;
    
    if(vqf_GetRestDetected()){  //静止判定フラグ
        BVC.BVC_debug_flag |= FLAG_BVC_REST_DETECT;
    }
}

void BodyVelocityControl_2(void){
    if(MC_state.state != MC_STATE_RUN){
        // 速度指示値
        BVC.vel_cmd[X_AXIS] = 0.0f;
        BVC.vel_cmd[Y_AXIS] = 0.0f;
        BVC.vel_cmd[Z_AXIS] = 0.0f;
        BVC.vel_cmd_dbg[X_AXIS] = 0.0f;
        BVC.vel_cmd_dbg[Y_AXIS] = 0.0f;
        BVC.vel_cmd_dbg[Z_AXIS] = 0.0f;
        BVC.vel_ref[X_AXIS] = 0.0f;
        BVC.vel_ref[Y_AXIS] = 0.0f;
        BVC.vel_ref[Z_AXIS] = 0.0f;
        BVC.vel_res[X_AXIS] = 0.0f;
        BVC.vel_res[Y_AXIS] = 0.0f;
        BVC.vel_res[Z_AXIS] = 0.0f;
        BVC.vel_res_z1[X_AXIS] = 0.0f;
        BVC.vel_res_z1[Y_AXIS] = 0.0f;
        BVC.vel_res_z1[Z_AXIS] = 0.0f;
        BVC.pos_res[X_AXIS] = 0.0f;
        BVC.pos_res[Y_AXIS] = 0.0f;
        BVC.pos_res[Z_AXIS] = 0.0f;
        BVC.acc_lpf[Z_AXIS] = 0.0f;
        BVC.acc_dis_lpf[X_AXIS] = 0.0f;
        BVC.acc_dis_lpf[Y_AXIS] = 0.0f;
        BVC.acc_dis_lpf[Z_AXIS] = 0.0f;
        return;
    }
    //APからの速度の指令値(m/s)を左右の車輪角速度(rad/s)->[Hz]に変換
    BVC.v_ref[SV1_FR] = (BVC.vel_ref[X_AXIS] + (MECP.tread/2.0f)*BVC.vel_ref[Z_AXIS])/(2*MECP.wheel_radius_r)/(SMC_PI_F)*SV1_DIRECTION;   //右車輪
    BVC.v_ref[SV2_FL] = (BVC.vel_ref[X_AXIS] - (MECP.tread/2.0f)*BVC.vel_ref[Z_AXIS])/(2*MECP.wheel_radius_l)/(SMC_PI_F)*SV2_DIRECTION;   //左車輪
}

void BodyVelocityControl(void){
    if(MC_state.state != MC_STATE_RUN){
        // 速度指示値
        BVC.vel_cmd[X_AXIS] = 0.0f;
        BVC.vel_cmd[Y_AXIS] = 0.0f;
        BVC.vel_cmd[Z_AXIS] = 0.0f;
        BVC.vel_cmd_dbg[X_AXIS] = 0.0f;
        BVC.vel_cmd_dbg[Y_AXIS] = 0.0f;
        BVC.vel_cmd_dbg[Z_AXIS] = 0.0f;
        BVC.vel_res[X_AXIS] = 0.0f;
        BVC.vel_res[Y_AXIS] = 0.0f;
        BVC.vel_res[Z_AXIS] = 0.0f;
        BVC.vel_res_z1[X_AXIS] = 0.0f;
        BVC.vel_res_z1[Y_AXIS] = 0.0f;
        BVC.vel_res_z1[Z_AXIS] = 0.0f;
        // BVC.pos_res[X_AXIS] = 0.0f;
        // BVC.pos_res[Y_AXIS] = 0.0f;
        // BVC.pos_res[Z_AXIS] = 0.0f;
        BVC.acc_lpf[Z_AXIS] = 0.0f;
        BVC.acc_dis_lpf[X_AXIS] = 0.0f;
        BVC.acc_dis_lpf[Y_AXIS] = 0.0f;
        BVC.acc_dis_lpf[Z_AXIS] = 0.0f;
        return;
    }

    // 関節角速度応答値
    BVC.dq_res[SV1_FR] = sv_res[SV1_FR].vel_res * SV1_DIRECTION;
    BVC.dq_res[SV2_FL] = sv_res[SV2_FL].vel_res * SV2_DIRECTION;
    //BVC.dq_res[SV3_HR] = sv_res[SV3_HR].vel_res * SV3_DIRECTION;
    //BVC.dq_res[SV4_HL] = sv_res[SV4_HL].vel_res * SV4_DIRECTION;

#if (HW_WHEEL == HW_WHEEL_MECANUM)
    // ヤコビ計算
    //ヤコビ
    float jaco_k = 0.25f * WHEEL_RADIUS;
    float jaco_l = BODY_HALF_LENGTH_X + BODY_HALF_LENGTH_Y;
    BVC.Jaco[X_AXIS][SV1_FR] = jaco_k;
    BVC.Jaco[X_AXIS][SV2_FL] = jaco_k;
    BVC.Jaco[X_AXIS][SV3_HR] = jaco_k;
    BVC.Jaco[X_AXIS][SV4_HL] = jaco_k;
    BVC.Jaco[Y_AXIS][SV1_FR] = - jaco_k;
    BVC.Jaco[Y_AXIS][SV2_FL] = jaco_k;
    BVC.Jaco[Y_AXIS][SV3_HR] = jaco_k;
    BVC.Jaco[Y_AXIS][SV4_HL] = - jaco_k;
    BVC.Jaco[Z_AXIS][SV1_FR] = jaco_k / jaco_l;
    BVC.Jaco[Z_AXIS][SV2_FL] = - jaco_k / jaco_l;
    BVC.Jaco[Z_AXIS][SV3_HR] = jaco_k / jaco_l;
    BVC.Jaco[Z_AXIS][SV4_HL] = - jaco_k / jaco_l;

    // ヤコビ逆
    float inv_jaco_k = 1.0f / WHEEL_RADIUS;
    BVC.invJaco[SV1_FR][X_AXIS] = inv_jaco_k;
    BVC.invJaco[SV1_FR][Y_AXIS] = - inv_jaco_k;
    BVC.invJaco[SV1_FR][Z_AXIS] = jaco_l* inv_jaco_k;
    BVC.invJaco[SV2_FL][X_AXIS] = inv_jaco_k;
    BVC.invJaco[SV2_FL][Y_AXIS] = inv_jaco_k;
    BVC.invJaco[SV2_FL][Z_AXIS] = - jaco_l* inv_jaco_k;
    BVC.invJaco[SV3_HR][X_AXIS] = inv_jaco_k;
    BVC.invJaco[SV3_HR][Y_AXIS] = inv_jaco_k;
    BVC.invJaco[SV3_HR][Z_AXIS] = jaco_l* inv_jaco_k;
    BVC.invJaco[SV4_HL][X_AXIS] = inv_jaco_k;
    BVC.invJaco[SV4_HL][Y_AXIS] = - inv_jaco_k;
    BVC.invJaco[SV4_HL][Z_AXIS] = - jaco_l* inv_jaco_k;

    // ヤコビ転置
    BVC.transJaco[SV1_FR][X_AXIS] = BVC.Jaco[X_AXIS][SV1_FR];
    BVC.transJaco[SV1_FR][Y_AXIS] = BVC.Jaco[Y_AXIS][SV1_FR];
    BVC.transJaco[SV1_FR][Z_AXIS] = BVC.Jaco[Z_AXIS][SV1_FR];
    BVC.transJaco[SV2_FL][X_AXIS] = BVC.Jaco[X_AXIS][SV2_FL];
    BVC.transJaco[SV2_FL][Y_AXIS] = BVC.Jaco[Y_AXIS][SV2_FL];
    BVC.transJaco[SV2_FL][Z_AXIS] = BVC.Jaco[Z_AXIS][SV2_FL];
    BVC.transJaco[SV3_HR][X_AXIS] = BVC.Jaco[X_AXIS][SV3_HR];
    BVC.transJaco[SV3_HR][Y_AXIS] = BVC.Jaco[Y_AXIS][SV3_HR];
    BVC.transJaco[SV3_HR][Z_AXIS] = BVC.Jaco[Z_AXIS][SV3_HR];
    BVC.transJaco[SV4_HL][X_AXIS] = BVC.Jaco[X_AXIS][SV4_HL];
    BVC.transJaco[SV4_HL][Y_AXIS] = BVC.Jaco[Y_AXIS][SV4_HL];
    BVC.transJaco[SV4_HL][Z_AXIS] = BVC.Jaco[Z_AXIS][SV4_HL];
    //ヤコビ転置逆

    //速度応答値 //低域をモータ角速度、高域をIMUを使ったフュージョン処理をつくる予定
    BVC.vel_res[X_AXIS] = BVC.Jaco[X_AXIS][SV1_FR] * BVC.dq_res[SV1_FR] + BVC.Jaco[X_AXIS][SV2_FL] * BVC.dq_res[SV2_FL] + BVC.Jaco[X_AXIS][SV3_HR] * BVC.dq_res[SV3_HR] + BVC.Jaco[X_AXIS][SV4_HL] * BVC.dq_res[SV4_HL];
    BVC.vel_res[Y_AXIS] = BVC.Jaco[Y_AXIS][SV1_FR] * BVC.dq_res[SV1_FR] + BVC.Jaco[Y_AXIS][SV2_FL] * BVC.dq_res[SV2_FL] + BVC.Jaco[Y_AXIS][SV3_HR] * BVC.dq_res[SV3_HR] + BVC.Jaco[Y_AXIS][SV4_HL] * BVC.dq_res[SV4_HL];
    BVC.vel_res[Z_AXIS] = BVC.Jaco[Z_AXIS][SV1_FR] * BVC.dq_res[SV1_FR] + BVC.Jaco[Z_AXIS][SV2_FL] * BVC.dq_res[SV2_FL] + BVC.Jaco[Z_AXIS][SV3_HR] * BVC.dq_res[SV3_HR] + BVC.Jaco[Z_AXIS][SV4_HL] * BVC.dq_res[SV4_HL];
    //BVC.vel_res[Z_AXIS] = imu_data.gyro[Z_AXIS];

    //位置応答値
    BVC.pos_res[X_AXIS] += (cosf(BVC.pos_res[Z_AXIS]) * (BVC.vel_res[X_AXIS] + BVC.vel_res_z1[X_AXIS]) + sinf(BVC.pos_res[Z_AXIS]) * (BVC.vel_res[Y_AXIS] + BVC.vel_res_z1[Y_AXIS])) * 0.5f / MOTION_CONTROL_FREQ;
    BVC.pos_res[Y_AXIS] += (sinf(BVC.pos_res[Z_AXIS]) * (BVC.vel_res[X_AXIS] + BVC.vel_res_z1[X_AXIS]) - cosf(BVC.pos_res[Z_AXIS]) * (BVC.vel_res[Y_AXIS] + BVC.vel_res_z1[Y_AXIS])) * 0.5f / MOTION_CONTROL_FREQ;;
    BVC.pos_res[Z_AXIS] += (BVC.vel_res[Z_AXIS] + BVC.vel_res_z1[Z_AXIS]) * 0.5f / MOTION_CONTROL_FREQ;

    BVC.vel_res_z1[X_AXIS] = BVC.vel_res[X_AXIS];
    BVC.vel_res_z1[Y_AXIS] = BVC.vel_res[Y_AXIS];
    BVC.vel_res_z1[Z_AXIS] = BVC.vel_res[Z_AXIS];

    //作業空間オブザーバ //あとで試してみる予定 //ふらふらする
    // BVC.acc_res[X_AXIS] = imu_data.acc[X_AXIS];
    // BVC.acc_res[Y_AXIS] = - imu_data.acc[Y_AXIS];
    // BVC.acc_res[Z_AXIS] = (imu_data.gyro[Z_AXIS] - BVC.acc_lpf[Z_AXIS]) * BVC.omega_wob;
    // BVC.acc_lpf[Z_AXIS] += 0.5f * 1.0f / MOTION_CONTROL_FREQ * (BVC.acc_res[Z_AXIS] + BVC.acc_res_z1[Z_AXIS]);
    // BVC.acc_res_z1[Z_AXIS] = BVC.acc_res[Z_AXIS];

    // BVC.acc_dis[X_AXIS] = BVC.acc_ref[X_AXIS] - BVC.acc_res[X_AXIS];
    // BVC.acc_dis[Y_AXIS] = BVC.acc_ref[Y_AXIS] - BVC.acc_res[Y_AXIS];
    // BVC.acc_dis[Z_AXIS] = BVC.acc_ref[Z_AXIS] - BVC.acc_res[Z_AXIS];

    // BVC.dot_acc_dis[X_AXIS] = (BVC.acc_dis[X_AXIS] - BVC.acc_dis_lpf[X_AXIS]) * BVC.omega_wob;
    // BVC.acc_dis_lpf[X_AXIS] += 0.5f * 1.0f / MOTION_CONTROL_FREQ * (BVC.dot_acc_dis[X_AXIS] + BVC.dot_acc_dis_z1[X_AXIS]);
    // BVC.dot_acc_dis_z1[X_AXIS] = BVC.dot_acc_dis[X_AXIS];

    // BVC.dot_acc_dis[Y_AXIS] = (BVC.acc_dis[Y_AXIS] - BVC.acc_dis_lpf[Y_AXIS]) * BVC.omega_wob;
    // BVC.acc_dis_lpf[Y_AXIS] += 0.5f * 1.0f / MOTION_CONTROL_FREQ * (BVC.dot_acc_dis[Y_AXIS] + BVC.dot_acc_dis_z1[Y_AXIS]);
    // BVC.dot_acc_dis_z1[Y_AXIS] = BVC.dot_acc_dis[Y_AXIS];

    // BVC.dot_acc_dis[Z_AXIS] = (BVC.acc_dis[Z_AXIS] - BVC.acc_dis_lpf[Z_AXIS]) * BVC.omega_wob;
    // BVC.acc_dis_lpf[Z_AXIS] += 0.5f * 1.0f / MOTION_CONTROL_FREQ * (BVC.dot_acc_dis[Z_AXIS] + BVC.dot_acc_dis_z1[Z_AXIS]);
    // BVC.dot_acc_dis_z1[Z_AXIS] = BVC.dot_acc_dis[Z_AXIS];

    //制御計算
    BVC.acc_ref[X_AXIS] = BVC.kp[X_AXIS] * (BVC.vel_cmd[X_AXIS] - BVC.vel_res[X_AXIS]);// + BVC.acc_dis_lpf[X_AXIS];
    BVC.acc_ref[Y_AXIS] = BVC.kp[Y_AXIS] * (BVC.vel_cmd[Y_AXIS] - BVC.vel_res[Y_AXIS]);// + BVC.acc_dis_lpf[Y_AXIS];
    BVC.acc_ref[Z_AXIS] = BVC.kp[Z_AXIS] * (BVC.vel_cmd[Z_AXIS] - BVC.vel_res[Z_AXIS]);// + BVC.acc_dis_lpf[Z_AXIS];//zだけ補償


    //分解加速度制御 ヤコビが定数なのでヤコビ時間微分が0
    BVC.ddq_ref[SV1_FR] = BVC.invJaco[SV1_FR][X_AXIS] * BVC.acc_ref[X_AXIS] + BVC.invJaco[SV1_FR][Y_AXIS] * BVC.acc_ref[Y_AXIS] + BVC.invJaco[SV1_FR][Z_AXIS] * BVC.acc_ref[Z_AXIS];
    BVC.ddq_ref[SV2_FL] = BVC.invJaco[SV2_FL][X_AXIS] * BVC.acc_ref[X_AXIS] + BVC.invJaco[SV2_FL][Y_AXIS] * BVC.acc_ref[Y_AXIS] + BVC.invJaco[SV2_FL][Z_AXIS] * BVC.acc_ref[Z_AXIS];
    BVC.ddq_ref[SV3_HR] = BVC.invJaco[SV3_HR][X_AXIS] * BVC.acc_ref[X_AXIS] + BVC.invJaco[SV3_HR][Y_AXIS] * BVC.acc_ref[Y_AXIS] + BVC.invJaco[SV3_HR][Z_AXIS] * BVC.acc_ref[Z_AXIS];
    BVC.ddq_ref[SV4_HL] = BVC.invJaco[SV4_HL][X_AXIS] * BVC.acc_ref[X_AXIS] + BVC.invJaco[SV4_HL][Y_AXIS] * BVC.acc_ref[Y_AXIS] + BVC.invJaco[SV4_HL][Z_AXIS] * BVC.acc_ref[Z_AXIS];


    //指示値
    BVC.tau_ref[SV1_FR] = BVC.ddq_ref[SV1_FR] * MOTOR_INERTIA;
    BVC.tau_ref[SV2_FL] = BVC.ddq_ref[SV2_FL] * MOTOR_INERTIA;
    BVC.tau_ref[SV3_HR] = BVC.ddq_ref[SV3_HR] * MOTOR_INERTIA;
    BVC.tau_ref[SV4_HL] = BVC.ddq_ref[SV4_HL] * MOTOR_INERTIA;


    BVC.i_ref[SV1_FR] = BVC.tau_ref[SV1_FR] / MOTOR_TORQUE_CONSTANT * SV1_DIRECTION;
    BVC.i_ref[SV2_FL] = BVC.tau_ref[SV2_FL] / MOTOR_TORQUE_CONSTANT * SV2_DIRECTION;
    BVC.i_ref[SV3_HR] = BVC.tau_ref[SV3_HR] / MOTOR_TORQUE_CONSTANT * SV3_DIRECTION;
    BVC.i_ref[SV4_HL] = BVC.tau_ref[SV4_HL] / MOTOR_TORQUE_CONSTANT * SV4_DIRECTION;

    for(int i = 0; i < 4; i++){
        if(BVC.i_ref[i] > MOTOR_CURRENT_LIMIT){
            BVC.i_ref[i] = MOTOR_CURRENT_LIMIT;
        }else if(BVC.i_ref[i] < - MOTOR_CURRENT_LIMIT){
            BVC.i_ref[i] = - MOTOR_CURRENT_LIMIT;
        }
    }
#elif (HW_WHEEL == HW_WHEEL_DIFF)
        // ヤコビ計算
    //ヤコビ
    float jaco_k = 0.5f * WHEEL_RADIUS;//0.5r
    float jaco_l = 2.0f * BODY_HALF_LENGTH_Y;//w
    BVC.Jaco[X_AXIS][SV1_FR] = jaco_k;
    BVC.Jaco[X_AXIS][SV2_FL] = jaco_k;
    BVC.Jaco[Y_AXIS][SV1_FR] = 0.0f;
    BVC.Jaco[Y_AXIS][SV2_FL] = 0.0f;
    BVC.Jaco[Z_AXIS][SV1_FR] = 2.0f * jaco_k / jaco_l;
    BVC.Jaco[Z_AXIS][SV2_FL] = - 2.0f * jaco_k / jaco_l;

    // ヤコビ逆
    float inv_jaco_k = 1.0f / WHEEL_RADIUS;//1/r
    BVC.invJaco[SV1_FR][X_AXIS] = inv_jaco_k;
    BVC.invJaco[SV1_FR][Y_AXIS] = 0.0f;
    BVC.invJaco[SV1_FR][Z_AXIS] = BODY_HALF_LENGTH_Y * inv_jaco_k;
    BVC.invJaco[SV2_FL][X_AXIS] = inv_jaco_k;
    BVC.invJaco[SV2_FL][Y_AXIS] = 0.0f;
    BVC.invJaco[SV2_FL][Z_AXIS] = - BODY_HALF_LENGTH_Y * inv_jaco_k;


    //ヤコビ転置逆

    //速度応答値 //低域をモータ角速度、高域をIMUを使ったフュージョン処理をつくる予定
    BVC.vel_res[X_AXIS] = BVC.Jaco[X_AXIS][SV1_FR] * BVC.dq_res[SV1_FR] + BVC.Jaco[X_AXIS][SV2_FL] * BVC.dq_res[SV2_FL];
    BVC.vel_res[Y_AXIS] = 0.0f;
    BVC.vel_res[Z_AXIS] = BVC.Jaco[Z_AXIS][SV1_FR] * BVC.dq_res[SV1_FR] + BVC.Jaco[Z_AXIS][SV2_FL] * BVC.dq_res[SV2_FL];
    //BVC.vel_res[Z_AXIS] = imu_data.gyro[Z_AXIS];

    //位置応答値
    BVC.pos_res[X_AXIS] += (cosf(BVC.pos_res[Z_AXIS]) * (BVC.vel_res[X_AXIS] + BVC.vel_res_z1[X_AXIS])) * 0.5f / MOTION_CONTROL_FREQ;
    BVC.pos_res[Y_AXIS] += (sinf(BVC.pos_res[Z_AXIS]) * (BVC.vel_res[X_AXIS] + BVC.vel_res_z1[X_AXIS])) * 0.5f / MOTION_CONTROL_FREQ;
    BVC.pos_res[Z_AXIS] += (BVC.vel_res[Z_AXIS] + BVC.vel_res_z1[Z_AXIS]) * 0.5f / MOTION_CONTROL_FREQ;

    if(BVC.pos_res[Z_AXIS] > 2.0f * 3.14159265358979f){
        BVC.pos_res[Z_AXIS] = BVC.pos_res[Z_AXIS] - 4.0f * 3.14159265358979f;
    }else if(BVC.pos_res[Z_AXIS] < - 2.0f * 3.14159265358979f){
        BVC.pos_res[Z_AXIS] = BVC.pos_res[Z_AXIS] + 4.0f * 3.14159265358979f;
    }

    BVC.vel_res_z1[X_AXIS] = BVC.vel_res[X_AXIS];
    BVC.vel_res_z1[Y_AXIS] = BVC.vel_res[Y_AXIS];
    BVC.vel_res_z1[Z_AXIS] = BVC.vel_res[Z_AXIS];

    //作業空間オブザーバ //あとで試してみる予定 //ふらふらする
    // BVC.acc_res[X_AXIS] = imu_data.acc[X_AXIS];
    // BVC.acc_res[Y_AXIS] = - imu_data.acc[Y_AXIS];
    // BVC.acc_res[Z_AXIS] = (imu_data.gyro[Z_AXIS] - BVC.acc_lpf[Z_AXIS]) * BVC.omega_wob;
    // BVC.acc_lpf[Z_AXIS] += 0.5f * 1.0f / MOTION_CONTROL_FREQ * (BVC.acc_res[Z_AXIS] + BVC.acc_res_z1[Z_AXIS]);
    // BVC.acc_res_z1[Z_AXIS] = BVC.acc_res[Z_AXIS];

    // BVC.acc_dis[X_AXIS] = BVC.acc_ref[X_AXIS] - BVC.acc_res[X_AXIS];
    // BVC.acc_dis[Y_AXIS] = BVC.acc_ref[Y_AXIS] - BVC.acc_res[Y_AXIS];
    // BVC.acc_dis[Z_AXIS] = BVC.acc_ref[Z_AXIS] - BVC.acc_res[Z_AXIS];

    // BVC.dot_acc_dis[X_AXIS] = (BVC.acc_dis[X_AXIS] - BVC.acc_dis_lpf[X_AXIS]) * BVC.omega_wob;
    // BVC.acc_dis_lpf[X_AXIS] += 0.5f * 1.0f / MOTION_CONTROL_FREQ * (BVC.dot_acc_dis[X_AXIS] + BVC.dot_acc_dis_z1[X_AXIS]);
    // BVC.dot_acc_dis_z1[X_AXIS] = BVC.dot_acc_dis[X_AXIS];

    // BVC.dot_acc_dis[Y_AXIS] = (BVC.acc_dis[Y_AXIS] - BVC.acc_dis_lpf[Y_AXIS]) * BVC.omega_wob;
    // BVC.acc_dis_lpf[Y_AXIS] += 0.5f * 1.0f / MOTION_CONTROL_FREQ * (BVC.dot_acc_dis[Y_AXIS] + BVC.dot_acc_dis_z1[Y_AXIS]);
    // BVC.dot_acc_dis_z1[Y_AXIS] = BVC.dot_acc_dis[Y_AXIS];

    // BVC.dot_acc_dis[Z_AXIS] = (BVC.acc_dis[Z_AXIS] - BVC.acc_dis_lpf[Z_AXIS]) * BVC.omega_wob;
    // BVC.acc_dis_lpf[Z_AXIS] += 0.5f * 1.0f / MOTION_CONTROL_FREQ * (BVC.dot_acc_dis[Z_AXIS] + BVC.dot_acc_dis_z1[Z_AXIS]);
    // BVC.dot_acc_dis_z1[Z_AXIS] = BVC.dot_acc_dis[Z_AXIS];

    //制御計算
    BVC.acc_ref[X_AXIS] = BVC.kp[X_AXIS] * (BVC.vel_cmd[X_AXIS] - BVC.vel_res[X_AXIS]);// + BVC.acc_dis_lpf[X_AXIS];
    BVC.acc_ref[Y_AXIS] = 0.0f;// + BVC.acc_dis_lpf[Y_AXIS];
    BVC.acc_ref[Z_AXIS] = BVC.kp[Z_AXIS] * (BVC.vel_cmd[Z_AXIS] - BVC.vel_res[Z_AXIS]);// + BVC.acc_dis_lpf[Z_AXIS];//zだけ補償


    //分解加速度制御 ヤコビが定数なのでヤコビ時間微分が0
    BVC.ddq_ref[SV1_FR] = BVC.invJaco[SV1_FR][X_AXIS] * BVC.acc_ref[X_AXIS] + BVC.invJaco[SV1_FR][Y_AXIS] * BVC.acc_ref[Y_AXIS] + BVC.invJaco[SV1_FR][Z_AXIS] * BVC.acc_ref[Z_AXIS];
    BVC.ddq_ref[SV2_FL] = BVC.invJaco[SV2_FL][X_AXIS] * BVC.acc_ref[X_AXIS] + BVC.invJaco[SV2_FL][Y_AXIS] * BVC.acc_ref[Y_AXIS] + BVC.invJaco[SV2_FL][Z_AXIS] * BVC.acc_ref[Z_AXIS];


    //指示値
    BVC.tau_ref[SV1_FR] = BVC.ddq_ref[SV1_FR] * MOTOR_INERTIA;
    BVC.tau_ref[SV2_FL] = BVC.ddq_ref[SV2_FL] * MOTOR_INERTIA;


    BVC.i_ref[SV1_FR] = BVC.tau_ref[SV1_FR] / MOTOR_TORQUE_CONSTANT * SV1_DIRECTION;
    BVC.i_ref[SV2_FL] = BVC.tau_ref[SV2_FL] / MOTOR_TORQUE_CONSTANT * SV2_DIRECTION;
    BVC.i_ref[SV3_HR] = 0.0f;
    BVC.i_ref[SV4_HL] = 0.0f;

    for(int i = 0; i < 4; i++){
        if(BVC.i_ref[i] > MOTOR_CURRENT_LIMIT){
            BVC.i_ref[i] = MOTOR_CURRENT_LIMIT;
        }else if(BVC.i_ref[i] < - MOTOR_CURRENT_LIMIT){
            BVC.i_ref[i] = - MOTOR_CURRENT_LIMIT;
        }
    }
#endif


}
